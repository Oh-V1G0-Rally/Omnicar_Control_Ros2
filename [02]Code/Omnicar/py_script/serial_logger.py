import serial
import serial.tools.list_ports
import time
import csv
import os
import datetime
import subprocess
import sys

# CONFIGURAZIONE
BAUD_RATE = 115200
# Cartella dove salvare i log (Percorso assoluto rispetto allo script)
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "csv_test")

def get_serial_port():
    """Trova automaticamente o chiede all'utente la porta seriale."""
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("Nessuna porta seriale trovata!")
        return None
    
    print("\n--- Porte Seriali Disponibili ---")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")
    
    # Se c'è una sola porta, usala direttamente (comodo per Raspberry/PC dedicati)
    if len(ports) == 1:
        print(f"Selezionata automaticamente: {ports[0].device}")
        return ports[0].device
    
    try:
        idx = int(input("\nSeleziona indice porta: "))
        return ports[idx].device
    except (ValueError, IndexError):
        print("Selezione non valida.")
        return None

def main():
    port = get_serial_port()
    if not port:
        return

    # Crea cartella output se non esiste
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    # Nome file con timestamp
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(OUTPUT_DIR, f"log_serial_{timestamp}.csv")
    
    print(f"\n--- In ascolto su {port} ({BAUD_RATE}) ---")
    print(f"--- Salvataggio in: {filename} ---")
    print("--- Premi Ctrl+C per terminare ---\n")

    try:
        # Configura la seriale disabilitando DTR/RTS per evitare reset indesiderati dell'ESP32
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        ser.dtr = False
        ser.rts = False
        ser.reset_input_buffer()

        with ser, open(filename, 'w', newline='') as csvfile:
            
            writer = None
            
            while True:
                try:
                    # Legge una riga dalla seriale (byte -> stringa)
                    line_bytes = ser.readline()
                    line = line_bytes.decode('utf-8', errors='replace').strip()
                    
                    if line:
                        print(f"[RX] {line}")
                        
                        # Se la riga contiene dati CSV (identificati dal separatore ';')
                        if ';' in line:
                            parts = line.split(';')
                            
                            # Inizializza il writer CSV alla prima riga valida (header o dati)
                            if writer is None:
                                writer = csv.writer(csvfile, delimiter=';')
                            
                            writer.writerow(parts)
                            csvfile.flush() # Scrive subito su disco
                            
                except UnicodeDecodeError:
                    pass # Ignora errori di decodifica sporadici

    except serial.SerialException as e:
        print(f"\n\n[ERRORE] Connessione persa o dispositivo disconnesso!\nMotivo: {e}")

    except KeyboardInterrupt:
        print(f"\nSTOP. File salvato: {filename}")
        
        # --- AUTO-LAUNCH PLOTTER ---
        plot_script = os.path.join(os.path.dirname(__file__), 'plot_motor_response.py')
        if os.path.exists(plot_script):
            print(f"Avvio grafico automatico: {plot_script}...")
            subprocess.run([sys.executable, plot_script, filename])
        # ---------------------------

if __name__ == "__main__":
    main()