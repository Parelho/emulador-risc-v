import os

# Define opcodes for RV32I instructions
OPCODES = {
    'LUI': 0b0110111,
    'AUIPC': 0b0010111,
    'JAL': 0b1101111,
    'JALR': 0b1100111,
    'BRANCH': 0b1100011,
    'LOAD': 0b0000011,
    'STORE': 0b0100011,
    'ALU_IMM': 0b0010011,
    'ALU_REG': 0b0110011,
}

# Define function codes for ALU operations
FUNCT3_CODES = {
    'ADD_SUB': 0b000,
    'SLL': 0b001,
    'SLT': 0b010,
    'SLTU': 0b011,
    'XOR': 0b100,
    'SRL_SRA': 0b101,
    'OR': 0b110,
    'AND': 0b111,
}

class RiscVEmulator:
    def __init__(self, memory_size=1024):
        # 32 general-purpose registers (x0-x31)
        self.registers = [0] * 32
        # Program counter
        self.pc = 0
        # Memory (initialized with a fixed size)
        self.memory = [0] * memory_size

    def load_program(self, program):
        """Load a list of instructions into memory starting at address 0."""
        self.memory[:len(program)] = program

    def fetch(self):
        """Fetch the instruction at the current program counter."""
        if self.pc >= len(self.memory) or self.pc % 4 != 0:
            raise IndexError("Program counter out of bounds or misaligned.")
        
        instruction = self.memory[self.pc // 4]  # Access the instruction
        self.pc += 4  # Increment pc by 4 for next instruction
        return instruction

    def decode(self, instruction):
        """Decode the instruction into opcode, rd, rs1, rs2, funct3, funct7, and immediate values."""
        opcode = instruction & 0x7F
        rd = (instruction >> 7) & 0x1F
        funct3 = (instruction >> 12) & 0x7
        rs1 = (instruction >> 15) & 0x1F
        rs2 = (instruction >> 20) & 0x1F
        funct7 = (instruction >> 25) & 0x7F

        # Default immediate value
        imm = 0
        
        # Handle immediate value extraction based on opcode
        if opcode in {OPCODES['STORE'], OPCODES['LOAD']}:
            imm = ((instruction >> 25) << 5) | ((instruction >> 7) & 0x1F)
        elif opcode == OPCODES['BRANCH']:
            imm = ((instruction >> 31) << 12) | ((instruction >> 7) & 0x1E) | \
                  ((instruction >> 25) << 5) | ((instruction >> 8) & 0xF)
        elif opcode == OPCODES['JAL']:
            imm = ((instruction >> 31) << 20) | ((instruction & 0xFF000) >> 12) | \
                  ((instruction >> 20) & 0x1) << 11 | (instruction >> 21) & 0x3FF
        
        return opcode, rd, funct3, rs1, rs2, funct7, imm

    def execute(self, opcode, rd, funct3, rs1, rs2, funct7, imm):
        """Execute the instruction based on opcode and decoded fields."""
        if opcode == OPCODES['ALU_REG']:
            if funct3 == FUNCT3_CODES['ADD_SUB']:
                if funct7 == 0b0000000:  # ADD rd,rs1,rs2
                    self.registers[rd] = self.registers[rs1] + self.registers[rs2]
                elif funct7 == 0b0100000:  # SUB rd,rs1,rs2
                    self.registers[rd] = self.registers[rs1] - self.registers[rs2]

    def run(self):
        """Run the emulator by fetching, decoding, and executing instructions."""
        try:
            while True:
                # Fetch instruction at the current program counter
                instruction = self.fetch()
                
                # Decode the fetched instruction
                opcode, rd, funct3, rs1, rs2, funct7, imm = self.decode(instruction)
                
                # Execute the instruction
                if opcode in OPCODES.values():
                    self.execute(opcode, rd, funct3, rs1, rs2, funct7, imm)
                else:
                    print(f"Unknown opcode: {opcode}. Halting.")
                    break
                
                # Stop if we reach a NOP
                if instruction == 0x00000013:  # NOP (0x00000013)
                    print("Encountered NOP. Halting.")
                    break
        except IndexError as e:
            print(f"Error: {e}. Halting.")
    
def test_riscv_emulator():
    emulator = RiscVEmulator()

    emulator.registers[0] = 0b00001010  # r0 = 10
    emulator.registers[1] = 0b00010100 # r1 = 20
    emulator.registers[2] = 0b00000101  # r2 = 5

    program = [
        (0b0000000 << 25) | (3 << 15) | (2 << 20) | (3 << 7) | 0b0110011,  # r3 = r2 + r3
        (0b0100000 << 25) | (1 << 15) | (5 << 20) | (4 << 7) | 0b0110011,  # r4 = r1 - r5
        0x00000013
    ]
    print("Before execution:")
    print(f"registers: {emulator.registers}\n")

    emulator.load_program(program)
    emulator.run()

    print("After execution:")
    print(f"registers: {emulator.registers}")

# Execute the testbench
if __name__ == "__main__":
    test_riscv_emulator()
