// n64_emulator.c
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

// ---------------------------------------------
// CONSTANTS
// ---------------------------------------------
#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480
#define RAM_SIZE (256 * 1024 * 1024) // 256 MB to accommodate higher address ranges
#define REG_COUNT 32                // MIPS CPU registers
#define CPU_FREQ 93750000           // ~93.75MHz (N64 CPU frequency)
#define INSTRUCTIONS_PER_FRAME (CPU_FREQ / 60) // Rough approximation for CPU cycles/frame

// ---------------------------------------------
// GLOBALS
// ---------------------------------------------
static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_AudioDeviceID audio_device;

// CPU State
uint32_t gpr[REG_COUNT];  // General-purpose registers
uint32_t pc;              // Program counter
uint32_t hi, lo;          // HI/LO registers for multiply/div
uint32_t next_pc;         // For branch delay slots

// Memory and ROM
uint8_t *rdram = NULL;    // RDRAM
bool rom_loaded = false;

// I/O Registers (very simplified)
uint8_t io_registers[256];

// PI, SI, VI, AI, RSP, RDP states (stubs)
typedef struct {
    // TODO: Add PI registers and DMA logic
} PIState;

typedef struct {
    // TODO: Add SI registers and controller pak/mempak logic
} SIState;

typedef struct {
    // VI (Video Interface) registers, timing, framebuffer pointers
    uint32_t vi_status;
    uint32_t vi_origin;
    uint32_t vi_width;
    uint32_t vi_intr;
    uint32_t vi_current;
    uint32_t vi_burst;
    uint32_t vi_vsync;
    uint32_t vi_hsync;
    uint32_t vi_leap;
    uint32_t vi_hstart;
    uint32_t vi_vstart;
    uint32_t vi_vburst;
    uint32_t vi_xscale;
    uint32_t vi_yscale;
} VIState;

typedef struct {
    // AI (Audio Interface)
    uint32_t ai_dram_addr;
    uint32_t ai_len;
    double   audio_position; // For a simulated tone
} AIState;

typedef struct {
    // RSP (Reality Signal Processor)
    // Executes microcode, handles geometry, transforms, and audio tasks.
} RSPState;

typedef struct {
    // RDP (Reality Display Processor)
    // Rasterizer: takes display lists from RSP and renders them.
} RDPState;

PIState pi;
SIState si;
VIState vi;
AIState ai;
RSPState rsp;
RDPState rdp;

// ---------------------------------------------
// Utility functions
// ---------------------------------------------

// Read 32 bits from memory with big-endian conversion
static inline uint32_t read32(uint32_t addr) {
    if (addr + 3 >= RAM_SIZE) {
        fprintf(stderr, "Memory read out of bounds: 0x%08X\n", addr);
        return 0;
    }
    return (rdram[addr] << 24) | (rdram[addr + 1] << 16) |
           (rdram[addr + 2] << 8) | rdram[addr + 3];
}

// Write 32 bits to memory with big-endian conversion
static inline void write32(uint32_t addr, uint32_t val) {
    if (addr + 3 >= RAM_SIZE) {
        fprintf(stderr, "Memory write out of bounds: 0x%08X\n", addr);
        return;
    }
    rdram[addr]     = (val >> 24) & 0xFF;
    rdram[addr + 1] = (val >> 16) & 0xFF;
    rdram[addr + 2] = (val >> 8) & 0xFF;
    rdram[addr + 3] = val & 0xFF;
}

// ---------------------------------------------
// ROM Loading
// ---------------------------------------------
int load_rom(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "Failed to open ROM file: %s\n", filename);
        return 0;
    }

    fseek(f, 0, SEEK_END);
    long rom_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (rom_size + 0x00000000 > RAM_SIZE) { // Load ROM at address 0x00000000
        fprintf(stderr, "ROM is too large.\n");
        fclose(f);
        return 0;
    }

    size_t bytes_read = fread(rdram, 1, rom_size, f);
    if (bytes_read != rom_size) {
        fprintf(stderr, "Failed to read the entire ROM.\n");
        fclose(f);
        return 0;
    }

    fclose(f);
    rom_loaded = true;
    pc = 0x00000000;      // Start PC at ROM start
    next_pc = pc + 4;
    printf("ROM loaded successfully. Size: %ld bytes\n", rom_size);
    return 1;
}

// ---------------------------------------------
// CPU Emulation (MIPS R4300i) - Enhanced
// ---------------------------------------------
// MIPS Instruction Fields
#define OPCODE(instr) ((instr >> 26) & 0x3F)
#define RS(instr)     ((instr >> 21) & 0x1F)
#define RT(instr)     ((instr >> 16) & 0x1F)
#define RD(instr)     ((instr >> 11) & 0x1F)
#define SHAMT(instr)  ((instr >> 6) & 0x1F)
#define FUNCT(instr)  (instr & 0x3F)
#define IMMED(instr)  ((uint16_t)(instr & 0xFFFF))
#define SIMMED(instr) ((int16_t)(instr & 0xFFFF))
#define TARGET(instr) (instr & 0x03FFFFFF)

// Register access with bounds checking
static inline void set_reg(uint8_t r, uint32_t val) {
    if (r != 0 && r < REG_COUNT) {
        gpr[r] = val;
    }
}

static inline uint32_t get_reg(uint8_t r) {
    if (r < REG_COUNT) {
        return gpr[r];
    }
    return 0;
}

// Memory read/write functions with address translation
uint32_t mem_read32_cpu(uint32_t addr) {
    // Simple direct mapping for demonstration
    return read32(addr);
}

void mem_write32_cpu(uint32_t addr, uint32_t val) {
    write32(addr, val);
}

void execute_instruction(uint32_t instr) {
    uint8_t opcode = OPCODE(instr);
    uint8_t rs_reg = RS(instr);
    uint8_t rt_reg = RT(instr);
    uint8_t rd_reg = RD(instr);
    uint8_t shamt  = SHAMT(instr);
    uint8_t funct  = FUNCT(instr);
    int16_t simm   = SIMMED(instr);
    uint16_t uimm  = IMMED(instr);
    uint32_t base_addr, val;

    switch (opcode) {
        case 0x00: // SPECIAL
            switch (funct) {
                case 0x20: // ADD
                    set_reg(rd_reg, (int32_t)get_reg(rs_reg) + (int32_t)get_reg(rt_reg));
                    break;
                case 0x22: // SUB
                    set_reg(rd_reg, (int32_t)get_reg(rs_reg) - (int32_t)get_reg(rt_reg));
                    break;
                case 0x24: // AND
                    set_reg(rd_reg, get_reg(rs_reg) & get_reg(rt_reg));
                    break;
                case 0x25: // OR
                    set_reg(rd_reg, get_reg(rs_reg) | get_reg(rt_reg));
                    break;
                case 0x2A: // SLT
                    set_reg(rd_reg, ((int32_t)get_reg(rs_reg) < (int32_t)get_reg(rt_reg)) ? 1 : 0);
                    break;
                // ... More R-type instructions ...
                default:
                    fprintf(stderr, "Unimplemented SPECIAL funct: 0x%02X at PC: 0x%08X\n", funct, pc);
                    break;
            }
            break;
        case 0x08: // ADDI
            set_reg(rt_reg, (int32_t)get_reg(rs_reg) + simm);
            break;
        case 0x0D: // ORI
            set_reg(rt_reg, get_reg(rs_reg) | uimm);
            break;
        case 0x23: // LW
            base_addr = get_reg(rs_reg) + simm;
            val = mem_read32_cpu(base_addr);
            set_reg(rt_reg, val);
            break;
        case 0x2B: // SW
            base_addr = get_reg(rs_reg) + simm;
            mem_write32_cpu(base_addr, get_reg(rt_reg));
            break;
        case 0x04: // BEQ
            if (get_reg(rs_reg) == get_reg(rt_reg)) {
                next_pc = pc + 4 + ((int32_t)simm << 2);
            }
            break;
        case 0x05: // BNE
            if (get_reg(rs_reg) != get_reg(rt_reg)) {
                next_pc = pc + 4 + ((int32_t)simm << 2);
            }
            break;
        case 0x02: // J
            next_pc = ((pc + 4) & 0xF0000000) | (TARGET(instr) << 2);
            break;
        case 0x03: // JAL
            set_reg(31, pc + 8); // Link register
            next_pc = ((pc + 4) & 0xF0000000) | (TARGET(instr) << 2);
            break;
        // ... Implement full instruction set ...
        default:
            fprintf(stderr, "Unimplemented opcode: 0x%02X at PC: 0x%08X\n", opcode, pc);
            break;
    }

    pc = next_pc;
    // next_pc is already updated based on instruction type
}

void emulate_cpu(int cycles) {
    for (int i = 0; i < cycles; i++) {
        if (!rom_loaded) {
            // If no ROM, just increment PC
            pc += 4;
            next_pc = pc + 4;
            continue;
        }

        uint32_t instr = mem_read32_cpu(pc);
        execute_instruction(instr);
    }
}

// ---------------------------------------------
// RSP Emulation (Stub)
void emulate_rsp() {
    // TODO: Implement RSP microcode execution
}

// ---------------------------------------------
// RDP Emulation (Stub)
void emulate_rdp() {
    // TODO: Implement RDP command decoding and rasterization
}

// ---------------------------------------------
// AI (Audio) Emulation
void emulate_ai() {
    // Simulate a simple sine wave tone
    ai.audio_position += 1.0;
    if (ai.audio_position > 44100.0) {
        ai.audio_position -= 44100.0;
    }
}

// ---------------------------------------------
// Video Interface (VI)
void emulate_vi() {
    // TODO: Implement VI timing, interrupts, and framebuffer updates
}

// ---------------------------------------------
// Input Handling
void write_io_register(uint8_t address, uint8_t value) {
    if (address < sizeof(io_registers)) {
        io_registers[address] = value;
    }
}

uint8_t read_io_register(uint8_t address) {
    if (address < sizeof(io_registers)) {
        return io_registers[address];
    }
    return 0;
}

void handle_input(SDL_Event *event) {
    if (event->type == SDL_KEYDOWN || event->type == SDL_KEYUP) {
        bool pressed = (event->type == SDL_KEYDOWN);
        switch (event->key.keysym.sym) {
            case SDLK_UP:
                write_io_register(0x01, pressed ? 0x01 : 0x00);
                break;
            case SDLK_DOWN:
                write_io_register(0x02, pressed ? 0x01 : 0x00);
                break;
            case SDLK_LEFT:
                write_io_register(0x03, pressed ? 0x01 : 0x00);
                break;
            case SDLK_RIGHT:
                write_io_register(0x04, pressed ? 0x01 : 0x00);
                break;
            case SDLK_a:
                write_io_register(0x05, pressed ? 0x01 : 0x00);
                break;
            case SDLK_b:
                write_io_register(0x06, pressed ? 0x01 : 0x00);
                break;
            // Add more controls as needed
            default:
                break;
        }
    }
}

// ---------------------------------------------
// Rendering Graphics
void render_graphics() {
    // Clear the screen with a solid color (e.g., blue)
    SDL_SetRenderDrawColor(renderer, 0, 0, 50, 255);
    SDL_RenderClear(renderer);

    // Placeholder: Draw a white rectangle representing the framebuffer
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_Rect rect = { SCREEN_WIDTH / 4, SCREEN_HEIGHT / 4,
                     SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 };
    SDL_RenderFillRect(renderer, &rect);

    SDL_RenderPresent(renderer);
}

// ---------------------------------------------
// Audio Callback
void audio_callback(void *userdata, Uint8 *stream, int len) {
    // Produce a sine wave tone
    static double phase = 0.0;
    int16_t *out = (int16_t*)stream;
    int samples = len / 4; // 2 channels, 16-bit
    double freq = 440.0; // A4 pitch
    double increment = (2.0 * M_PI * freq) / 44100.0;

    for (int i = 0; i < samples; i++) {
        int16_t sample = (int16_t)(sin(phase) * 3000);
        out[i * 2]     = sample; // Left channel
        out[i * 2 + 1] = sample; // Right channel
        phase += increment;
        if (phase > 2.0 * M_PI) {
            phase -= 2.0 * M_PI;
        }
    }
}

// ---------------------------------------------
// Initialization and Cleanup
int init_sdl() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS) < 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 0;
    }

    window = SDL_CreateWindow("N64 Emulator - Optimized Framework",
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 0;
    }

    renderer = SDL_CreateRenderer(window, -1,
                                  SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return 0;
    }

    SDL_AudioSpec want, have;
    SDL_zero(want);
    want.freq = 44100;
    want.format = AUDIO_S16SYS;
    want.channels = 2;
    want.samples = 512;
    want.callback = audio_callback;

    audio_device = SDL_OpenAudioDevice(NULL, 0, &want, &have, SDL_AUDIO_ALLOW_FORMAT_CHANGE);
    if (audio_device == 0) {
        fprintf(stderr, "Failed to open audio device: %s\n", SDL_GetError());
    } else {
        SDL_PauseAudioDevice(audio_device, 0);
    }

    return 1;
}

int init_emulator(const char *rom_path) {
    rdram = (uint8_t *)aligned_alloc(16, RAM_SIZE); // Ensure alignment for SIMD operations
    if (!rdram) {
        fprintf(stderr, "Failed to allocate %d MB of RDRAM\n", RAM_SIZE / (1024 * 1024));
        return 0;
    }
    memset(rdram, 0, RAM_SIZE);
    memset(io_registers, 0, sizeof(io_registers));
    memset(gpr, 0, sizeof(gpr));
    pc = 0x00000000; 
    next_pc = pc + 4;
    hi = lo = 0;

    if (rom_path && !load_rom(rom_path)) {
        return 0;
    }

    // Initialize VI, AI, PI, SI, RDP, RSP states (not implemented in detail)
    ai.audio_position = 0.0;

    return 1;
}

void cleanup_sdl() {
    if (audio_device != 0) {
        SDL_CloseAudioDevice(audio_device);
    }
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
}

void cleanup_emulator() {
    if (rdram) free(rdram);
    rdram = NULL;
}

// ---------------------------------------------
// Main Loop
// ---------------------------------------------
void emulator_loop() {
    bool running = true;
    SDL_Event event;
    uint32_t frame_start, frame_time;
    const int target_fps = 60;
    const int frame_delay = 1000 / target_fps;

    while (running) {
        frame_start = SDL_GetTicks();

        // Handle Input
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else {
                handle_input(&event);
            }
        }

        // Emulate CPU - execute instructions based on cycles per frame
        emulate_cpu(INSTRUCTIONS_PER_FRAME / target_fps);

        // Emulate other components
        emulate_rsp();
        emulate_rdp();
        emulate_ai();
        emulate_vi();

        // Render graphics
        render_graphics();

        // Frame timing to maintain target FPS
        frame_time = SDL_GetTicks() - frame_start;
        if (frame_delay > frame_time) {
            SDL_Delay(frame_delay - frame_time);
        }
    }
}

// ---------------------------------------------
// Main Entry Point
// ---------------------------------------------
int main(int argc, char *argv[]) {
    const char *rom_path = NULL;
    if (argc > 1) {
        rom_path = argv[1];
    } else {
        fprintf(stderr, "Usage: %s <path_to_rom>\n", argv[0]);
        return EXIT_FAILURE;
    }

    if (!init_sdl()) {
        cleanup_sdl();
        return EXIT_FAILURE;
    }

    if (!init_emulator(rom_path)) {
        cleanup_sdl();
        cleanup_emulator();
        return EXIT_FAILURE;
    }

    printf("Starting emulator loop...\n");
    emulator_loop();

    cleanup_emulator();
    cleanup_sdl();
    return EXIT_SUCCESS;
}
