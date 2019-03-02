#include "cpu.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

// Define which register to store Stack Pointer
#define SP_REG 7
// Define which register handles interrupt status
#define IS_REG 6
// Define which register handles interrupt mask
#define IM_REG 5
// Define which bit is LGE. Helps when comparing Flag register
#define LESS 0b00000100
#define GREATER 0b00000010
#define EQUAL 0b00000001

// Initilize int to keep track of length of program in RAM
// Used to trigger stack overflow warning
unsigned int loaded_ram_address = 0;

////////////////////
// CPU Helper Functions
////////////////////

// Return value at RAM address
unsigned char cpu_ram_read(struct cpu *cpu, unsigned char address)
{
  return cpu->ram[address];
}

// Write value at RAM address
void cpu_ram_write(struct cpu *cpu, unsigned char address, unsigned char val)
{
  cpu->ram[address] = val;
};

// Push value to top of stack
void push(struct cpu *cpu, unsigned char val)
{
  // Decrement stack pointer and check for overflow
  if (cpu->reg[SP_REG]-- <= loaded_ram_address)
  {
    fprintf(stderr, "Stack overflow.\n");
    exit(4);
  }
  // Write to stack
  cpu_ram_write(cpu, cpu->reg[SP_REG], val);
}

// Return value at top of stack
unsigned char pop(struct cpu *cpu)
{
  // Get value at stack pointer and increment
  unsigned char value = cpu_ram_read(cpu, cpu->reg[SP_REG]++);
  // Check if stack is empty
  if (cpu->reg[SP_REG] > 244)
  {
    fprintf(stderr, "Stack underflow.\n");
    exit(4);
  }
  // return value
  return value;
}

void handle_interrupt(struct cpu *cpu)
{
  // 1. The IM register is bitwise AND-ed with the IS register and the results stored as `maskedInterrupts`.
  unsigned int maskedInterrupts = cpu->reg[IS_REG] & cpu->reg[IM_REG];
  unsigned int check_bit = 0x01;
  // 2. Each bit of `maskedInterrupts` is checked, starting from 0 and going up to the 7th bit, one for each interrupt.
  for (int i = 0; i < 8; i++)
  {
    // If interrupt found
    if (maskedInterrupts & check_bit)
    {
      // 1. Change interrupt flag - disabling interrupts until IRET
      cpu->interrupt_fl = 1;
      // 2. Clear bit in the IS register
      cpu->reg[IS_REG] = cpu->reg[IS_REG] ^ check_bit;
      // 3. Push PC on stack
      push(cpu, cpu->PC);
      // 4. Push FL register
      // push(cpu, cpu->FL);
      // 5. Push registers 0-6 in order
      for (int j = 0; j < 7; j++)
      {
        push(cpu, cpu->reg[j]);
      }
      // 6. The address (_vector_ in interrupt terminology) of the appropriate handler is looked up from the interrupt vector table.
      unsigned int vector = 0b11111000 | i;
      // 7. Set the PC is set to the handler address.
      cpu->PC = cpu_ram_read(cpu, vector);
      // break for loop at bit handling first interrupt
      break;
    }
    // else shift bit to check next one
    check_bit = check_bit << 1;
  }
  // return to cpu_run
}

///////////////////////
// CPU Helper Functions End
///////////////////////

/**
 * Load the binary bytes from a .ls8 source file into a RAM array
 */
void cpu_load(struct cpu *cpu, char *file)
{
  // Access file
  FILE *fp;
  // Initialize buffer to read line by line
  char line_buf[1024];
  // Initialize starting address
  loaded_ram_address = 0;
  // Check file exists
  if ((fp = fopen(file, "r")) == NULL)
  {
    fprintf(stderr, "File doesn't exist\n");
    exit(2);
  }

  // While line exists
  while (fgets(line_buf, sizeof(line_buf), fp) != NULL)
  {
    // initialize pointer to store string part of line
    char *ptr;
    // initialize int to store number part of line
    unsigned int ret;
    // convert string to number
    ret = strtol(line_buf, &ptr, 2);

    // If string equal to line (ie. blank) go to next line
    if (ptr == line_buf)
    {
      continue;
    }
    // store converted number in ram and increment address
    cpu->ram[loaded_ram_address++] = ret;
  }
  // Close file
  fclose(fp);
}

/**
 * ALU
 */
void alu(struct cpu *cpu, enum alu_op op, unsigned char regA, unsigned char regB)
{
  switch (op)
  {
  case ALU_ADD:
    // Add the value in two registers and store the result in registerA.
    cpu->reg[regA] += cpu->reg[regB];
    break;
  case ALU_AND:
    //Bitwise-AND the values in registerA and registerB, then store the result in registerA.
    cpu->reg[regA] = cpu->reg[regA] & cpu->reg[regB];
    break;
  case ALU_CMP:
    // Compare the values in two registers.
    // Set FL to 0
    cpu->FL = cpu->FL & 0x00;
    // if A less than B set flag L 00000100 = 4
    if (cpu->reg[regA] < cpu->reg[regB])
    {
      cpu->FL = cpu->FL | LESS;
    }
    // if A greater than B set flag G 00000010 = 2
    else if (cpu->reg[regA] > cpu->reg[regB])
    {
      cpu->FL = cpu->FL | GREATER;
    }
    // else if equal set flag E 00000001 = 1
    else
    {
      cpu->FL = cpu->FL | EQUAL;
    }
    break;
  case ALU_DEC:
    // Decrement (subtract 1 from) the value in the given register.
    cpu->reg[regA]--;
    break;
  case ALU_DIV:
    // Divide the value in the first register by the value in the second, storing the result in registerA.
    cpu->reg[regA] /= cpu->reg[regB];
    break;
  case ALU_INC:
    // Increment (add 1 to) the value in the given register.
    cpu->reg[regA]++;
    break;
  case ALU_MOD:
    // Divide the value in the first register by the value in the second, storing the _remainder_ of the result in registerA.
    cpu->reg[regA] %= cpu->reg[regB];
    break;
  case ALU_MUL:
    // Multiply the values in two registers together and store the result in registerA.
    cpu->reg[regA] *= cpu->reg[regB];
    break;
  case ALU_NOT:
    // Perform a bitwise-NOT on the value in a register.
    cpu->reg[regA] = ~cpu->reg[regA];
    break;
  case ALU_OR:
    // Perform a bitwise-OR between the values in registerA and registerB, storing the result in registerA.
    cpu->reg[regA] |= cpu->reg[regB];
  case ALU_SHL:
    // Shift the value in registerA left by the number of bits specified in registerB, filling the low bits with 0.
    cpu->reg[regA] = cpu->reg[regA] << cpu->reg[regB];
    break;
  case ALU_SHR:
    // Shift the value in registerA right by the number of bits specified in registerB, filling the high bits with 0.
    cpu->reg[regA] = cpu->reg[regA] >> cpu->reg[regB];
    break;
  case ALU_SUB:
    // Subtract the value in the second register from the first, storing the result in registerA.
    cpu->reg[regA] -= cpu->reg[regB];
    break;
  case ALU_XOR:
    //Perform a bitwise-XOR between the values in registerA and registerB, storing the result in registerA.
    cpu->reg[regA] = cpu->reg[regA] ^ cpu->reg[regB];
    break;
  default:
    break;
  }
}

//////////////////////////
// Handle IR Funcs Start
//////////////////////////
// Functions that all return void and implement individual instructions
void handle_ADD(struct cpu *cpu, unsigned char operandA, unsigned char operandB)
{ // *This is an instruction handled by the ALU.*
  alu(cpu, ALU_ADD, operandA, operandB);
}
void handle_AND(struct cpu *cpu, unsigned char operandA, unsigned char operandB)
{ // *This is an instruction handled by the ALU.*
  alu(cpu, ALU_AND, operandA, operandB);
}
void handle_CALL(struct cpu *cpu, unsigned char operandA)
{ // Calls a subroutine (function) at the address stored in the register.
  // 1. Store address of next instruction after CALL on stack
  push(cpu, (cpu->PC + 2));
  // 2. PC is set to address stored in given register
  cpu->PC = cpu->reg[operandA];
}
void handle_CMP(struct cpu *cpu, unsigned char operandA, unsigned char operandB)
{ // *This is an instruction handled by the ALU.*
  alu(cpu, ALU_CMP, operandA, operandB);
}
void handle_DEC(struct cpu *cpu, unsigned char operandA, unsigned char operandB)
{ // *This is an instruction handled by the ALU.*
  alu(cpu, ALU_DEC, operandA, operandB);
}
void handle_DIV(struct cpu *cpu, unsigned char operandA, unsigned char operandB)
{ // *This is an instruction handled by the ALU.*
  // If the value in the second register is 0, the system should print an error message and halt.
  if (cpu->reg[operandB] == 0)
  {
    printf("Can not divide by 0\n");
    cpu->running = 0;
  }
  else
  {
    alu(cpu, ALU_DIV, operandA, operandB);
  }
}
void handle_HLT(struct cpu *cpu) { cpu->running = 0; }
void handle_INC(struct cpu *cpu, unsigned char operandA, unsigned char operandB)
{ // *This is an instruction handled by the ALU.*
  alu(cpu, ALU_INC, operandA, operandB);
}
void handle_INT(struct cpu *cpu, unsigned char operandA)
{ // Issue the interrupt number stored in the given register.
  // This will set the _n_th bit in the `IS` register to the value in the given register.
  cpu->reg[IS_REG] |= cpu->reg[operandA];
}
void handle_IRET(struct cpu *cpu)
{
  for (int i = 6; i >= 0; i--)
  {
    cpu->reg[i] = pop(cpu);
  }
  // cpu->FL = pop(cpu);
  cpu->PC = pop(cpu);
  cpu->interrupt_fl = 0;
}
void handle_JEQ(struct cpu *cpu, unsigned char operandA)
{ // If `equal` flag is set (true), jump to the address stored in the given register.
  if (cpu->FL & EQUAL)
  {
    cpu->PC = cpu->reg[operandA];
  }
  // Else manually increment PC if it isn't set
  else
  {
    cpu->PC += 2;
  }
}
void handle_JGE(struct cpu *cpu, unsigned char operandA)
{ // If `greater-than` flag or `equal` flag is set (true), jump to the address stored in the given register.
  if (cpu->FL & (GREATER | EQUAL))
  {
    cpu->PC = cpu->reg[operandA];
  }
  // Else manually increment PC if it isn't set
  else
  {
    cpu->PC += 2;
  }
}
void handle_JGT(struct cpu *cpu, unsigned char operandA)
{ // If `greater-than` flag is set (true), jump to the address stored in the given register.
  if (cpu->FL & GREATER)
  {
    cpu->PC = cpu->reg[operandA];
  }
  // Else manually increment PC if it isn't set
  else
  {
    cpu->PC += 2;
  }
}
void handle_JLE(struct cpu *cpu, unsigned char operandA)
{ // If `less-than` flag or `equal` flag is set (true), jump to the address stored in the given register.
  if (cpu->FL & (LESS | EQUAL))
  {
    cpu->PC = cpu->reg[operandA];
  }
  // Else manually increment PC if it isn't set
  else
  {
    cpu->PC += 2;
  }
}
void handle_JLT(struct cpu *cpu, unsigned char operandA)
{ // If `less-than` flag is set (true), jump to the address stored in the given register.
  if (cpu->FL & LESS)
  {
    cpu->PC = cpu->reg[operandA];
  }
  // Else manually increment PC if it isn't set
  else
  {
    cpu->PC += 2;
  }
}
void handle_JMP() {}

//////////////////////////
// Handle IR Funcs End
//////////////////////////

/**
 * Run the CPU
 */
void cpu_run(struct cpu *cpu)
{
  cpu->running = 1; // True until we get a HLT instruction
  unsigned char operandA;
  unsigned char operandB;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  unsigned int prev_sec = tv.tv_sec;
  cpu->interrupt_fl = 0;

  while (cpu->running)
  {
    // 1. Get the value of the current instruction (in address PC). Store in Instruction Register or IR
    unsigned char instruction = cpu_ram_read(cpu, cpu->PC);
    // 2. Determine how many operands this next instruction requires from bits 6-7 of instruction opcode
    unsigned int num_operands = instruction >> 6;
    // 3. Get the appropriate value(s) of the operands following this instruction
    operandA = cpu_ram_read(cpu, cpu->PC + 1);
    operandB = cpu_ram_read(cpu, cpu->PC + 2);

    // Check for time interrupt
    gettimeofday(&tv, NULL);
    // printf("tv: %ld  prev_sec: %u\n", tv.tv_sec, prev_sec);
    if (tv.tv_sec != prev_sec)
    {
      cpu->reg[IS_REG] |= 0x01;
    }
    prev_sec = tv.tv_sec;

    // printf("TRACE: %02X: %02X   %02X %02X\n", cpu->PC, instruction, operandA, operandB);

    // 4. switch() over it to decide on a course of action.
    // 5. Do whatever the instruction should do according to the spec.
    switch (instruction)
    {
    case ADD:
      handle_ADD(cpu, operandA, operandB);
      break;
    case AND:
      handle_AND(cpu, operandA, operandB);
      break;
    case CALL:
      handle_CALL(cpu, operandA);
      break;
    case CMP:
      handle_CMP(cpu, operandA, operandB);
      break;
    case DEC:
      handle_DEC(cpu, operandA, operandB);
      break;
    case DIV:
      handle_DIV(cpu, operandA, operandB);
      break;
    case HLT:
      handle_HLT(cpu);
      break;
    case INC:
      handle_INC(cpu, operandA, operandB);
      break;
    case INT:
      handle_INT(cpu, operandA);
      break;
    case IRET:
      handle_IRET(cpu);
      break;
    case JEQ:
      handle_JEQ(cpu, operandA);
      break;
    case JGE:
      handle_JGE(cpu, operandA);
      break;
    case JGT:
      handle_JGT(cpu, operandA);
      break;
    case JLE:
      handle_JLE(cpu, operandA);
      break;
    case JLT:
      handle_JLT(cpu, operandA);
      break;
    case JMP:
      // Jump to the address stored in the given register.
      // Set the `PC` to the address stored in the given register.
      cpu->PC = cpu->reg[operandA];
      break;
    case JNE:
      //If `E` flag is clear (false, 0), jump to the address stored in the given register.
      // Bitwise & last bit. If not set then jump.
      if ((cpu->FL & EQUAL) == 0)
      {
        cpu->PC = cpu->reg[operandA];
      }
      // Else manually increment PC if it isn't set
      else
      {
        cpu->PC += num_operands + 1;
      }
      break;
    case LD:
      // Loads registerA with the value at the memory address stored in registerB.
      cpu->reg[operandA] = cpu_ram_read(cpu, cpu->reg[operandB]);
      break;
    case LDI:
      // Set the value of a register to an integer.
      cpu->reg[operandA] = operandB;
      break;
    case MOD:
      // *This is an instruction handled by the ALU.*
      // If the value in the second register is 0, the system should print an error message and halt.
      if (cpu->reg[operandB] == 0)
      {
        printf("Can not mod by 0\n");
        cpu->running = 0;
      }
      else
      {
        alu(cpu, ALU_MOD, operandA, operandB);
      }
      break;
    case MUL:
      // *This is an instruction handled by the ALU.*
      alu(cpu, ALU_MUL, operandA, operandB);
      break;
    case NOP:
      // No operation. Do nothing for this instruction.
      break;
    case NOT:
      // *This is an instruction handled by the ALU.*
      alu(cpu, ALU_NOT, operandA, operandB);
      break;
    case OR:
      // *This is an instruction handled by the ALU.*
      alu(cpu, ALU_OR, operandA, operandB);
      break;
    case POP:
      // Pop the value at the top of the stack into the given register.
      cpu->reg[operandA] = pop(cpu);
      break;
    case PRA:
      // Print alpha character value stored in the given register.
      printf("%c\n", cpu->reg[operandA]);
      break;
    case PRN:
      // Print numeric value stored in the given register.
      printf("%d\n", cpu->reg[operandA]);
      break;
    case PUSH:
      // Push the value in the given register on the stack.
      push(cpu, cpu->reg[operandA]);
      break;
    case RET:
      // Return from subroutine.
      // Pop the value from the top of the stack and store it in the `PC`.
      cpu->PC = pop(cpu);
      break;
    case SHL:
      // *This is an instruction handled by the ALU.*
      alu(cpu, ALU_SHL, operandA, operandB);
      break;
    case SHR:
      // *This is an instruction handled by the ALU.*
      alu(cpu, ALU_SHR, operandA, operandB);
      break;
    case ST:
      // Store value in registerB in the address stored in registerA.
      cpu_ram_write(cpu, cpu->reg[operandA], cpu->reg[operandB]);
      break;
    case SUB:
      // *This is an instruction handled by the ALU.*
      alu(cpu, ALU_SUB, operandA, operandB);
      break;
    case XOR:
      // *This is an instruction handled by the ALU.*
      alu(cpu, ALU_XOR, operandA, operandB);
      break;
    default:
      printf("unexpected instruction 0x%02X at 0x%02X\n", instruction, cpu->PC);
      exit(3);
    }
    // 6. Check to see if instruction sets PC. Move the PC to the next instruction if needed.
    int PC_set = instruction >> 4 & 0x01;
    if (PC_set == 0)
    {
      cpu->PC += num_operands + 1;
    }
    // 7. Check for interrupt and check if not already handling an interrupt
    if ((cpu->reg[IS_REG] != 0) && cpu->interrupt_fl == 0)
    {
      handle_interrupt(cpu);
    }
  }
}

/**
 * Initialize a CPU struct
 */
void cpu_init(struct cpu *cpu)
{
  //Initialize the PC and other special registers
  cpu->PC = 0;
  memset(cpu->reg, 0, sizeof(cpu->reg));
  memset(cpu->ram, 0, sizeof(cpu->ram));
  // Empty stack starts at address F4
  cpu->reg[SP_REG] = 0xF4;
  // Initialize FL
  cpu->FL = 0;
}
