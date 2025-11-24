#!/usr/bin/env python3
"""
Простой ассемблер для нашего микроконтроллера
"""

class Assembler:
    def __init__(self):
        self.instructions = {
            'NOP': 0x00,
            'LDI': 0x01,
            'MOV': 0x02,
            'ADD': 0x03,
            'SUB': 0x04,
            'JMP': 0x05,
            'JZ': 0x06,
            'JNZ': 0x07,
            'CALL': 0x08,
            'RET': 0x09,
            'PUSH': 0x0A,
            'POP': 0x0B,
            'AND': 0x0C,
            'OR': 0x0D,
            'XOR': 0x0E,
            'HALT': 0x0F,
        }
    
    def parse_register(self, reg_str):
        """Парсинг регистра: R0 -> 0, R1 -> 1, etc."""
        if reg_str.startswith('R') and reg_str[1:].isdigit():
            return int(reg_str[1:])
        raise ValueError(f"Неверный регистр: {reg_str}")
    
    def parse_number(self, num_str):
        """Парсинг чисел: 10, 0x0A, $0A -> 10"""
        if num_str.startswith('0x'):
            return int(num_str[2:], 16)
        elif num_str.startswith('$'):
            return int(num_str[1:], 16)
        else:
            return int(num_str)
    
    def assemble(self, code):
        """Ассемблирование исходного кода в машинный код"""
        lines = code.strip().split('\n')
        machine_code = []
        labels = {}
        
        # Первый проход: сбор меток
        address = 0
        for line in lines:
            line = line.split(';')[0].strip()  # Удаляем комментарии
            if not line:
                continue
                
            if line.endswith(':'):
                # Это метка
                label = line[:-1]
                labels[label] = address
            else:
                # Это инструкция
                parts = line.split()
                if parts:
                    address += 1  # opcode
                    # Добавляем байты для операндов
                    if parts[0] in ['LDI', 'MOV', 'ADD', 'SUB', 'AND', 'OR', 'XOR']:
                        address += 2
                    elif parts[0] in ['JMP', 'JZ', 'JNZ', 'CALL']:
                        address += 2
                    elif parts[0] in ['PUSH', 'POP']:
                        address += 1
        
        # Второй проход: генерация кода
        address = 0
        for line in lines:
            line = line.split(';')[0].strip()
            if not line or line.endswith(':'):
                continue
                
            parts = line.split()
            opcode_str = parts[0]
            
            if opcode_str not in self.instructions:
                raise ValueError(f"Неизвестная инструкция: {opcode_str}")
            
            opcode = self.instructions[opcode_str]
            machine_code.append(opcode)
            address += 1
            
            if opcode_str == 'LDI':
                # LDI Rd, imm
                rd = self.parse_register(parts[1].rstrip(','))
                imm = self.parse_number(parts[2])
                machine_code.extend([rd, imm])
                address += 2
                
            elif opcode_str in ['MOV', 'ADD', 'SUB', 'AND', 'OR', 'XOR']:
                # INSTR Rd, Rs
                rd = self.parse_register(parts[1].rstrip(','))
                rs = self.parse_register(parts[2])
                machine_code.extend([rd, rs])
                address += 2
                
            elif opcode_str in ['JMP', 'JZ', 'JNZ', 'CALL']:
                # INSTR addr
                addr_str = parts[1]
                if addr_str in labels:
                    addr = labels[addr_str]
                else:
                    addr = self.parse_number(addr_str)
                # Младший байт first (little-endian)
                machine_code.append(addr & 0xFF)
                machine_code.append((addr >> 8) & 0xFF)
                address += 2
                
            elif opcode_str in ['PUSH', 'POP']:
                # PUSH Rs или POP Rd
                reg = self.parse_register(parts[1])
                machine_code.append(reg)
                address += 1
        
        return machine_code

def test_assembler():
    """Тестирование ассемблера"""
    asm = Assembler()
    
    test_code = """
    ; Простая тестовая программа
    LDI R1, 10     ; Загрузить 10 в R1
    LDI R2, 20     ; Загрузить 20 в R2
    ADD R1, R2     ; R1 = R1 + R2
    MOV R3, R1     ; Скопировать результат в R3
    HALT           ; Остановка
    """
    
    try:
        machine_code = asm.assemble(test_code)
        print("Исходный код:")
        print(test_code)
        print("Машинный код:")
        print(" ".join(f"{b:02X}" for b in machine_code))
        return machine_code
    except Exception as e:
        print(f"Ошибка ассемблирования: {e}")
        return None

if __name__ == "__main__":
    test_assembler()