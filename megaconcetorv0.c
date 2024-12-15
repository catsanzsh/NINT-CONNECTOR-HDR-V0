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
#define RAM_SIZE (64 * 1024 * 1024) // 64 MB RDRAM
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
    // Actual N64 VI scales and filters the rendered image.
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
    // Handles reading audio data from RDRAM and outputting it.
    // For simplicity, we will just generate silence or a simple tone.
    uint32_t ai_dram_addr;
    uint32_t ai_len;
    double   audio_position; // For a simulated tone
} AIState;

typedef struct {
    // RSP (Reality Signal Processor)
    // Executes microcode, handles geometry, transforms, and audio tasks.
    // This would require implementing RSP vector instructions.
} RSPState;

typedef struct {
    // RDP (Reality Display Processor)
    // Rasterizer: takes display lists from RSP and renders them.
    // This requires implementing the RDP command set.
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
static inline uint32_t read32(uint8_t *mem, uint32_t addr) {
    return (mem[addr] << 24) | (mem[addr+1] << 16) | (mem[addr+2] << 8) | (mem[addr+3]);
}

static inline void write32(uint8_t *mem, uint32_t addr, uint32_t val) {
    mem[addr]   = (val >> 24) & 0xFF;
    mem[addr+1] = (val >> 16) & 0xFF;
    mem[addr+2] = (val >> 8) & 0xFF;
    mem[addr+3] = val & 0xFF;
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

    // For simplicity, load at 0x10000000. Real N64 ROMs are accessed via PI.
    if ((0x10000000 + rom_size) > RAM_SIZE) {
        fprintf(stderr, "ROM is too large.\n");
        fclose(f);
        return 0;
    }

    fread(rdram + 0x10000000, 1, rom_size, f);
    fclose(f);

    rom_loaded = true;
    return 1;
}

// ---------------------------------------------
// CPU Emulation (MIPS R4300i) - Partial Example
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

static inline void set_reg(uint8_t r, uint32_t val) {
    if (r != 0) gpr[r] = val;
}

uint32_t mem_read32_cpu(uint32_t addr) {
    // Real implementation: Check address ranges for RDRAM, I/O, ROM, etc.
    addr &= (RAM_SIZE - 1);
    return read32(rdram, addr);
}

void mem_write32_cpu(uint32_t addr, uint32_t val) {
    addr &= (RAM_SIZE - 1);
    write32(rdram, addr, val);
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

    // This is a tiny subset of possible instructions.
    // A full implementation would decode all MIPS R4300i instructions.
    switch (opcode) {
        case 0x00: // SPECIAL
            switch (funct) {
                case 0x20: // ADD
                    set_reg(rd_reg, (int32_t)gpr[rs_reg] + (int32_t)gpr[rt_reg]);
                    break;
                case 0x22: // SUB
                    set_reg(rd_reg, (int32_t)gpr[rs_reg] - (int32_t)gpr[rt_reg]);
                    break;
                case 0x24: // AND
                    set_reg(rd_reg, gpr[rs_reg] & gpr[rt_reg]);
                    break;
                case 0x25: // OR
                    set_reg(rd_reg, gpr[rs_reg] | gpr[rt_reg]);
                    break;
                // ... More R-type instructions ...
                default:
                    // Unimplemented
                    break;
            }
            break;
        case 0x08: // ADDI
            set_reg(rt_reg, (int32_t)gpr[rs_reg] + simm);
            break;
        case 0x0D: // ORI
            set_reg(rt_reg, gpr[rs_reg] | uimm);
            break;
        case 0x23: // LW
            base_addr = gpr[rs_reg] + simm;
            val = mem_read32_cpu(base_addr);
            set_reg(rt_reg, val);
            break;
        case 0x2B: // SW
            base_addr = gpr[rs_reg] + simm;
            mem_write32_cpu(base_addr, gpr[rt_reg]);
            break;
        // ... Implement full instruction set ...
        default:
            // Unimplemented instruction
            break;
    }

    pc = next_pc;
    next_pc += 4; // Default increment for next instruction
}

void emulate_cpu(int cycles) {
    // Execute a given number of instructions (cycles) per frame
    for (int i = 0; i < cycles; i++) {
        if (!rom_loaded) {
            // If no ROM, just increment PC
            pc += 4;
            next_pc = pc + 4;
            continue;
        }

        uint32_t instr = mem_read32_cpu(pc);
        next_pc = pc + 4;
        execute_instruction(instr);
    }
}

// ---------------------------------------------
// RSP Emulation (Stub)
void emulate_rsp() {
    // In reality, RSP runs microcode.
    // Here we do nothing.
}

// ---------------------------------------------
// RDP Emulation (Stub)
void emulate_rdp() {
    // In a real implementation, decode RDP commands from command buffers in RDRAM.
    // Rasterize triangles, apply texture filtering, etc.
}

// ---------------------------------------------
// AI (Audio) Emulation (Stub)
void emulate_ai() {
    // This would fetch audio samples from RDRAM and output them.
    // We will just increment a position to simulate a tone.
    ai.audio_position += 1.0;
}

// ---------------------------------------------
// Video Interface (VI)
void emulate_vi() {
    // Update VI counters, interrupts, etc.
}

// ---------------------------------------------
// Input handling
void write_io_register(uint8_t address, uint8_t value) {
    if (address < 256) {
        io_registers[address] = value;
    }
}

uint8_t read_io_register(uint8_t address) {
    if (address < 256) {
        return io_registers[address];
    }
    return 0;
}

void handle_input(SDL_Event *event) {
    if (event->type == SDL_KEYDOWN) {
        switch (event->key.keysym.sym) {
            case SDLK_UP: write_io_register(0x01, 0x01); break;
            case SDLK_DOWN: write_io_register(0x02, 0x01); break;
            case SDLK_LEFT: write_io_register(0x03, 0x01); break;
            case SDLK_RIGHT: write_io_register(0x04, 0x01); break;
            case SDLK_a: write_io_register(0x05, 0x01); break;
            default: break;
        }
    } else if (event->type == SDL_KEYUP) {
        switch (event->key.keysym.sym) {
            case SDLK_UP: write_io_register(0x01, 0x00); break;
            case SDLK_DOWN: write_io_register(0x02, 0x00); break;
            case SDLK_LEFT: write_io_register(0x03, 0x00); break;
            case SDLK_RIGHT: write_io_register(0x04, 0x00); break;
            case SDLK_a: write_io_register(0x05, 0x00); break;
            default: break;
        }
    }
}

// ---------------------------------------------
// Rendering graphics (Stub)
void render_graphics() {
    // In a real emulator, you'd convert the RDP framebuffer to a bitmap and render it.
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // White rectangle placeholder for "graphics"
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_Rect rect = { SCREEN_WIDTH / 4, SCREEN_HEIGHT / 4, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 };
    SDL_RenderFillRect(renderer, &rect);

    SDL_RenderPresent(renderer);
}

// ---------------------------------------------
// Audio callback (stub)
void audio_callback(void *userdata, Uint8 *stream, int len) {
    // Very simplistic: produce a sine wave tone
    // In a real emulator, we'd pull samples from AI buffers in RDRAM.
    static double phase = 0.0;
    int16_t *out = (int16_t*)stream;
    int samples = len / 4; // 2 channels, 16-bit
    double freq = 440.0; // A4 pitch
    double increment = (2.0 * M_PI * freq) / 44100.0;

    for (int i = 0; i < samples; i++) {
        int16_t sample = (int16_t)(sin(phase) * 3000);
        out[i*2] = sample;     // left
        out[i*2+1] = sample;   // right
        phase += increment;
    }
}

// ---------------------------------------------
// Initialization and Cleanup
int init_sdl() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS) < 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 0;
    }

    window = SDL_CreateWindow("N64 Emulator - Expanded Framework", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 0;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
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

    audio_device = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (audio_device == 0) {
        fprintf(stderr, "Failed to open audio device: %s\n", SDL_GetError());
    } else {
        SDL_PauseAudioDevice(audio_device, 0);
    }

    return 1;
}

int init_emulator(const char *rom_path) {
    rdram = (uint8_t *)malloc(RAM_SIZE);
    if (!rdram) {
        fprintf(stderr, "Failed to allocate 64 MB of RDRAM\n");
        return 0;
    }
    memset(rdram, 0, RAM_SIZE);
    memset(io_registers, 0, sizeof(io_registers));
    memset(gpr, 0, sizeof(gpr));
    pc = 0x10000000; 
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
    int running = 1;
    SDL_Event event;

    // Run until window close
    while (running) {
        // Handle Input
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            } else {
                handle_input(&event);
            }
        }

        // Emulate CPU - run a batch of instructions each frame
        emulate_cpu(100000); // Arbitrary number, real timing is complex

        // Emulate RSP
        emulate_rsp();

        // Emulate RDP
        emulate_rdp();

        // Emulate AI
        emulate_ai();

        // Emulate VI
        emulate_vi();

        // Render graphics
        render_graphics();

        // Delay to approx 60fps
        SDL_Delay(16);
    }
}

// ---------------------------------------------
// Main Entry Point
// ---------------------------------------------
int main(int argc, char *argv[]) {
    const char *rom_path = NULL;
    if (argc > 1) {
        rom_path = argv[1];
    }

    if (!init_sdl()) {
        return -1;
    }

    if (!init_emulator(rom_path)) {
        cleanup_sdl();
        return -1;
    }

    emulator_loop();

    cleanup_emulator();
    cleanup_sdl();
    return 0;
}
