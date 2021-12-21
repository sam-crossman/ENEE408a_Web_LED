import machine
import hashlib

def uart_run(mode):
    print("entered UART function...")
    uart_conn = machine.UART(2, 115200)
    uart_conn.init(115200, bits=8, parity=None, stop=1)
    # use a newline character and readline()???
    if (mode == 'red'):
        uart_conn.write('red')
    if (mode == 'green'):
        uart_conn.write('green')
    if (mode == 'blue'):
        uart_conn.write('blue')
    if (mode == 'white'):
        uart_conn.write('white')
    if (mode == 'yellow'):
        uart_conn.write('yellow')
    if (mode == 'purple'):
        uart_conn.write('purple')
    if (mode == 'check'):
        uart_conn.write('check')
    



