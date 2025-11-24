#!/usr/bin/env python3
"""
Эмулятор 8-битного микроконтроллера
Архитектура: 8-битная, 16 регистров, 64KB памяти
"""

import sys
import time

class Microcontroller:
    def __init__(self):
        # Регистры общего назначения R0-R15
        self.registers = [0] * 16
        
        # Специальные регистры
        self.pc = 0x0000  # Program Counter
        self.sp = 0xFFFF  # Stack Pointer
        self.status = 0   # Status Register
        
        # Память: 64KB
        self.memory = [0] * 65536
        
        # Флаги статусного регистра
        self.FLAG_Z = 1 << 0  # Zero flag
        self.FLAG_C = 1 << 1  # Carry flag
        self.FLAG_N = 1 << 2  # Negative flag
        
        # Таблица инструкций
        self.instructions = {
            0x00: self._nop,      # NOP
            0x01: self._ldi,      # LDI Rd, imm
            0x02: self._mov,      # MOV Rd, Rs
            0x03: self._add,      # ADD Rd, Rs
            0x04: self._sub,      # SUB Rd, Rs
            0x05: self._jmp,      # JMP addr
            0x06: self._jz,       # JZ addr
            0x07: self._jnz,      # JNZ addr
            0x08: self._call,     # CALL addr
            0x09: self._ret,      # RET
            0x0A: self._push,     # PUSH Rs
            0x0B: self._pop,      # POP Rd
            0x0C: self._and,      # AND Rd, Rs
            0x0D: self._or,       # OR Rd, Rs
            0x0E: self._xor,      # XOR Rd, Rs
            0x0F: self._halt,     # HALT
        }
        
        self.running = False
    
    def reset(self):
        """Сброс микроконтроллера"""
        self.registers = [0] * 16
        self.pc = 0x0000
        self.sp = 0xFFFF
        self.status = 0
        self.memory = [0] * 65536
        self.running = False
    
    def load_firmware(self, firmware_data, offset=0):
        """Загрузка прошивки в память"""
        for i, byte in enumerate(firmware_data):
            if offset + i < len(self.memory):
                self.memory[offset + i] = byte & 0xFF
    
    def fetch_byte(self):
        """Чтение байта из памяти и увеличение PC"""
        byte = self.memory[self.pc]
        self.pc = (self.pc + 1) & 0xFFFF
        return byte
    
    def fetch_word(self):
        """Чтение слова (2 байта) из памяти"""
        low = self.fetch_byte()
        high = self.fetch_byte()
        return (high << 8) | low
    
    def update_flags(self, result):
        """Обновление флагов на основе результата операции"""
        # Zero flag
        if result == 0:
            self.status |= self.FLAG_Z
        else:
            self.status &= ~self.FLAG_Z
        
        # Negative flag (для знаковых чисел)
        if result & 0x80:  # Проверка старшего бита
            self.status |= self.FLAG_N
        else:
            self.status &= ~self.FLAG_N
        
        # Carry flag сбрасывается здесь, устанавливается в отдельных операциях
        self.status &= ~self.FLAG_C
    
    # Реализации инструкций
    def _nop(self):
        """NOP - нет операции"""
        pass
    
    def _ldi(self):
        """LDI Rd, imm - загрузка непосредственного значения в регистр"""
        rd = self.fetch_byte() & 0x0F
        imm = self.fetch_byte()
        self.registers[rd] = imm
        self.update_flags(self.registers[rd])
    
    def _mov(self):
        """MOV Rd, Rs - копирование между регистрами"""
        rd = self.fetch_byte() & 0x0F
        rs = self.fetch_byte() & 0x0F
        self.registers[rd] = self.registers[rs]
        self.update_flags(self.registers[rd])
    
    def _add(self):
        """ADD Rd, Rs - сложение регистров"""
        rd = self.fetch_byte() & 0x0F
        rs = self.fetch_byte() & 0x0F
        result = self.registers[rd] + self.registers[rs]
        
        # Проверка на переполнение
        if result > 0xFF:
            self.status |= self.FLAG_C
            result &= 0xFF
        else:
            self.status &= ~self.FLAG_C
        
        self.registers[rd] = result
        self.update_flags(self.registers[rd])
    
    def _sub(self):
        """SUB Rd, Rs - вычитание регистров"""
        rd = self.fetch_byte() & 0x0F
        rs = self.fetch_byte() & 0x0F
        result = self.registers[rd] - self.registers[rs]
        
        # Проверка на заем
        if result < 0:
            self.status |= self.FLAG_C
            result &= 0xFF
        else:
            self.status &= ~self.FLAG_C
        
        self.registers[rd] = result
        self.update_flags(self.registers[rd])
    
    def _jmp(self):
        """JMP addr - безусловный переход"""
        addr = self.fetch_word()
        self.pc = addr
    
    def _jz(self):
        """JZ addr - переход если zero flag установлен"""
        addr = self.fetch_word()
        if self.status & self.FLAG_Z:
            self.pc = addr
    
    def _jnz(self):
        """JNZ addr - переход если zero flag не установлен"""
        addr = self.fetch_word()
        if not (self.status & self.FLAG_Z):
            self.pc = addr
    
    def _call(self):
        """CALL addr - вызов подпрограммы"""
        addr = self.fetch_word()
        # Сохраняем адрес возврата в стек
        self.sp = (self.sp - 1) & 0xFFFF
        self.memory[self.sp] = (self.pc >> 8) & 0xFF
        self.sp = (self.sp - 1) & 0xFFFF
        self.memory[self.sp] = self.pc & 0xFF
        self.pc = addr
    
    def _ret(self):
        """RET - возврат из подпрограммы"""
        low = self.memory[self.sp]
        self.sp = (self.sp + 1) & 0xFFFF
        high = self.memory[self.sp]
        self.sp = (self.sp + 1) & 0xFFFF
        self.pc = (high << 8) | low
    
    def _push(self):
        """PUSH Rs - поместить регистр в стек"""
        rs = self.fetch_byte() & 0x0F
        self.sp = (self.sp - 1) & 0xFFFF
        self.memory[self.sp] = self.registers[rs]
    
    def _pop(self):
        """POP Rd - извлечь значение из стека в регистр"""
        rd = self.fetch_byte() & 0x0F
        self.registers[rd] = self.memory[self.sp]
        self.sp = (self.sp + 1) & 0xFFFF
        self.update_flags(self.registers[rd])
    
    def _and(self):
        """AND Rd, Rs - побитовое И"""
        rd = self.fetch_byte() & 0x0F
        rs = self.fetch_byte() & 0x0F
        self.registers[rd] &= self.registers[rs]
        self.update_flags(self.registers[rd])
    
    def _or(self):
        """OR Rd, Rs - побитовое ИЛИ"""
        rd = self.fetch_byte() & 0x0F
        rs = self.fetch_byte() & 0x0F
        self.registers[rd] |= self.registers[rs]
        self.update_flags(self.registers[rd])
    
    def _xor(self):
        """XOR Rd, Rs - побитовое исключающее ИЛИ"""
        rd = self.fetch_byte() & 0x0F
        rs = self.fetch_byte() & 0x0F
        self.registers[rd] ^= self.registers[rs]
        self.update_flags(self.registers[rd])
    
    def _halt(self):
        """HALT - остановка процессора"""
        self.running = False
    
    def step(self):
        """Выполнение одной инструкции"""
        if not self.running:
            return False
        
        opcode = self.fetch_byte()
        
        if opcode in self.instructions:
            self.instructions[opcode]()
        else:
            print(f"Неизвестная инструкция: 0x{opcode:02X} по адресу 0x{(self.pc-1):04X}")
            self.running = False
            return False
        
        return True
    
    def run(self, start_addr=0x0000):
        """Запуск выполнения программы"""
        self.pc = start_addr
        self.running = True
        
        print("Запуск эмулятора...")
        cycle_count = 0
        
        while self.running and cycle_count < 1000:  # Защита от бесконечного цикла
            if not self.step():
                break
            cycle_count += 1
        
        if cycle_count >= 1000:
            print("Достигнут предел циклов выполнения")
        
        print(f"Выполнено циклов: {cycle_count}")
        self.print_state()
    
    def print_state(self):
        """Вывод состояния микроконтроллера"""
        print("\n" + "="*50)
        print("СОСТОЯНИЕ МИКРОКОНТРОЛЛЕРА")
        print("="*50)
        
        print("Регистры:")
        for i in range(0, 16, 4):
            regs = [f"R{i+j}: 0x{self.registers[i+j]:02X} ({self.registers[i+j]})" 
                   for j in range(4)]
            print("  " + " | ".join(regs))
        
        print(f"\nСпециальные регистры:")
        print(f"  PC: 0x{self.pc:04X}")
        print(f"  SP: 0x{self.sp:04X}")
        print(f"  STATUS: 0x{self.status:02X} [Z:{(self.status>>0)&1} C:{(self.status>>1)&1} N:{(self.status>>2)&1}]")
        
        print("\nПамять (первые 32 байта):")
        for i in range(0, 32, 16):
            hex_bytes = " ".join(f"{self.memory[i+j]:02X}" for j in range(16))
            ascii_repr = "".join(chr(b) if 32 <= b <= 126 else "." for b in self.memory[i:i+16])
            print(f"  0x{i:04X}: {hex_bytes}  {ascii_repr}")

def test_emulator():
    """Тестирование эмулятора с простой программой"""
    # Создаем простую тестовую прошивку вручную
    # Программа: складывает 5 + 7 и сохраняет результат в R2
    
    # Коды инструкций:
    # LDI R1, 5  -> 01 01 05
    # LDI R2, 7  -> 01 02 07  
    # ADD R1, R2 -> 03 01 02
    # MOV R2, R1 -> 02 02 01
    # HALT       -> 0F
    
    test_firmware = [
        0x01, 0x01, 0x05,  # LDI R1, 5
        0x01, 0x02, 0x07,  # LDI R2, 7
        0x03, 0x01, 0x02,  # ADD R1, R2 (R1 = R1 + R2)
        0x02, 0x02, 0x01,  # MOV R2, R1 (R2 = R1)
        0x0F,              # HALT
    ]
    
    # Запускаем эмулятор
    mc = Microcontroller()
    mc.load_firmware(test_firmware)
    mc.run()
    
    # Проверяем результат
    expected = 12  # 5 + 7
    actual = mc.registers[2]
    
    print(f"\nТЕСТ: 5 + 7 = {expected}")
    print(f"РЕЗУЛЬТАТ: R2 = {actual}")
    
    if actual == expected:
        print("✅ ТЕСТ ПРОЙДЕН!")
        return True
    else:
        print("❌ ТЕСТ НЕ ПРОЙДЕН!")
        return False

if __name__ == "__main__":
    print("Эмулятор микроконтроллера")
    print("Запуск тестовой программы...")
    test_emulator()