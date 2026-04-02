import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import csv
import argparse
import sys
import os

# Numero di righe da saltare all'inizio del file PRIMA dell'intestazione
# SKIP_DATA_ROWS = 5 (Disabilitato per ricerca automatica)

# Variabile per il percorso del file CSV da aprire
# Esempio Linux: "/media/user/Storage/Desktop_Ale/OmniCar_Control/[02]Code/Omnicar/py_script/csv_test/M0_T1.csv"
FILE_FOLDER_PATH = '/media/user/Storage/Desktop_Ale/OmniCar_Control/[02]Code/Omnicar/py_script/csv_test/'
# FILE_NAME = 'PID_2.csv'
FILE_NAME = 'log_serial_20260312_142913.csv'
FILE_PATH = os.path.join(FILE_FOLDER_PATH, FILE_NAME)

def main():
    parser = argparse.ArgumentParser(description='Omnicar CSV Plotter')
    parser.add_argument('filename', nargs='?', default=FILE_PATH, help='Percorso del file CSV da analizzare')
    parser.add_argument('-s', '--separate', action='store_true', help='Genera 4 grafici separati (uno per motore)')
    parser.add_argument('-i', '--index', type=int, help='Indice motore specifico da analizzare (0=M0, 1=M1, 2=M2, 3=M3)')
    args = parser.parse_args()

    if not os.path.isfile(args.filename):
        print(f"Errore: Il file '{args.filename}' non esiste.")
        print("Assicurati di aver specificato il percorso corretto.")
        sys.exit(1)

    print(f"--- ANALISI FILE: {args.filename} ---")
    
    # Liste per i dati
    times = []
    pwms = []
    motor_speeds = [[], [], [], []] # M0, M1, M2, M3
    headers = []

    try:
        with open(args.filename, 'r', newline='') as csvfile:
            # --- RICERCA AUTOMATICA INTESTAZIONE ---
            # Cerca la prima riga che contiene "Time_ms" invece di saltare righe fisse
            header_line = ""
            while True:
                line = csvfile.readline()
                if not line: break # Fine file
                if "Time_ms" in line:
                    header_line = line
                    break
            # ---------------------------------------

            if not header_line:
                print("Errore: Il file non contiene dati validi dopo le righe saltate.")
                sys.exit(1)

            try:
                # Usa csv.Sniffer per identificare automaticamente il delimitatore
                dialect = csv.Sniffer().sniff(header_line, delimiters=';,')
                detected_delimiter = dialect.delimiter
            except csv.Error:
                detected_delimiter = ';' if ';' in header_line else ','
            print(f"Delimitatore rilevato: '{detected_delimiter}'")

            # Parsiamo l'intestazione dalla riga appena letta
            headers = next(csv.reader([header_line], delimiter=detected_delimiter))
            print(f"Intestazioni rilevate: {headers}")
            
            # Rilevamento formato (Standard 6 col vs Legacy 3 col)
            is_legacy_single = False
            if len(headers) == 3:
                print("-> Rilevato formato Legacy (Singolo Motore). Adattamento in corso...")
                is_legacy_single = True

            # Crea il reader per il resto del file (parte dalla posizione corrente)
            reader = csv.reader(csvfile, delimiter=detected_delimiter)

            # Lettura righe
            for i, row in enumerate(reader):
                if not row: continue # Salta righe vuote
                try:
                    if is_legacy_single:
                        # Formato Legacy: Time_ms,PWM_Ref,Speed_rad_s
                        if len(row) < 3: continue
                        t = float(row[0].replace(',', '.'))
                        ref_rads = float(row[1].replace(',', '.'))
                        m0 = float(row[2].replace(',', '.'))
                        m1 = m2 = m3 = 0.0 # Altri motori a 0
                    else:
                        # Struttura Standard: Time_ms;Ref_Rads;Spd_M0;Spd_M1;Spd_M2;Spd_M3
                        if len(row) < 6: continue
                        t = float(row[0].replace(',', '.'))
                        ref_rads = float(row[1].replace(',', '.'))
                        m0 = float(row[2].replace(',', '.'))
                        m1 = float(row[3].replace(',', '.'))
                        m2 = float(row[4].replace(',', '.'))
                        m3 = float(row[5].replace(',', '.'))
                    
                    times.append(t)
                    pwms.append(ref_rads)
                    motor_speeds[0].append(m0)
                    motor_speeds[1].append(m1)
                    motor_speeds[2].append(m2)
                    motor_speeds[3].append(m3)

                except (ValueError, IndexError):
                    # Ignora righe che non contengono numeri validi (es. fine file o errori)
                    continue

    except Exception as e:
        print(f"Errore durante la lettura del file: {e}")
        sys.exit(1)

    if not times:
        print("Nessun dato valido trovato nel file.")
        sys.exit(1)

    # --- CREAZIONE GRAFICO ---
    print(f"Generazione grafico con {len(times)} punti...")
    
    if args.index is not None:
        # Modalità Motore Singolo Specifico (-i)
        if not (0 <= args.index < 4):
            print(f"Errore: Indice {args.index} non valido. Inserire un valore tra 0 e 3.")
            sys.exit(1)
            
        idx = args.index
        fig, ax1 = plt.subplots(figsize=(10, 6))
        
        color_speed = 'tab:blue'
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel(f'Speed M{idx} (rad/s)', color=color_speed)
        ax1.plot(times, motor_speeds[idx], color=color_speed, label=f'Speed M{idx}', linewidth=2)
        ax1.tick_params(axis='y', labelcolor=color_speed)
        ax1.grid(True, linestyle='--', alpha=0.6)
        
        # Asse Y2: ref_rads Reference
        ax2 = ax1.twinx()
        color_ref = 'black'
        ax2.set_ylabel('Reference (rad/s)', color=color_ref)
        ax2.plot(times, pwms, '--', color=color_ref, alpha=0.5, label='Reference')
        ax2.tick_params(axis='y', labelcolor=color_ref)
        
        plt.title(f"Analisi Dettaglio Motore {idx}: {os.path.basename(args.filename)}")
        
        # Legenda combinata
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

    elif args.separate:
        # Modalità 4 grafici separati
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"Analisi Motori Separata: {os.path.basename(args.filename)}")
        
        # Appiattisci l'array degli assi per iterare facilmente
        axs_flat = axs.flatten()
        
        for i in range(4):
            ax = axs_flat[i]
            color_speed = f'C{i}' # Colore diverso per ogni motore
            
            # Asse Y1: Velocità Motore
            ax.plot(times, motor_speeds[i], color=color_speed, label=f'Speed M{i}')
            ax.set_ylabel(f'Speed M{i} (rad/s)', color=color_speed)
            ax.tick_params(axis='y', labelcolor=color_speed)
            ax.grid(True, linestyle='--', alpha=0.6)
            
            # Asse Y2: ref_rads Reference
            ax2 = ax.twinx()
            ax2.plot(times, pwms, 'k--', alpha=0.3, label='rad/s Ref')
            if i % 2 == 1: # Solo sui grafici a destra per pulizia
                ax2.set_ylabel('rad/s Ref', color='k')
            
            ax.set_title(f'Motore {i}')
            
        # Etichetta asse X solo in basso
        for ax in axs.flat:
            ax.set_xlabel('Time (ms)')

    else:
        # Modalità grafico unico sovrapposto
        fig, ax1 = plt.subplots(figsize=(12, 6))
        
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel('Speed (rad/s)')
        
        for i in range(4):
            ax1.plot(times, motor_speeds[i], label=f'Speed M{i}', linewidth=1.5)
            
        ax1.legend(loc='upper left')
        ax1.grid(True, linestyle='--', alpha=0.6)
        
        # Asse Y2 per ref_rads
        ax2 = ax1.twinx()
        ax2.set_ylabel('rad/s Reference', color='k')
        ax2.plot(times, pwms, 'k--', linewidth=1, alpha=0.5, label='rad/s Ref')
        ax2.legend(loc='upper right')
        
        plt.title(f"Analisi Combinata Motori: {os.path.basename(args.filename)}")

    fig.tight_layout()
    # Lascia spazio in basso per il pulsante
    plt.subplots_adjust(bottom=0.15)

    # --- INTERATTIVITÀ: Cursore al click ---
    all_annotations = []
    
    # --- Pulsante CLEAR ---
    ax_btn = plt.axes([0.8, 0.02, 0.1, 0.06])
    btn_clear = Button(ax_btn, 'Clear Labels')

    def clear_labels(event):
        for annot in all_annotations:
            annot.remove()
        all_annotations.clear()
        fig.canvas.draw_idle()

    btn_clear.on_clicked(clear_labels)

    def on_click(event):
        # Ignora i click fuori dagli assi o sul pulsante stesso
        if event.inaxes is None or event.inaxes == ax_btn: return
        
        if not times: return
        
        # 1. Trova l'indice temporale più vicino
        idx = min(range(len(times)), key=lambda i: abs(times[i] - event.xdata))
        t_val = times[idx]
        
        # 2. Cerca la linea più vicina al click (distanza visiva in pixel)
        min_dist = 50.0
        best_candidate = None
        
        # Filtra gli assi per escludere il pulsante
        plot_axes = [ax for ax in fig.axes if ax != ax_btn]
        
        for ax in plot_axes:
            if ax.contains(event)[0]:
                for line in ax.lines:
                    y_data = line.get_ydata()
                    if len(y_data) != len(times): continue
                    
                    y_val = y_data[idx]
                    pt_screen = ax.transData.transform((t_val, y_val))
                    dist = ((pt_screen[0] - event.x)**2 + (pt_screen[1] - event.y)**2)**0.5
                    
                    if dist < min_dist:
                        min_dist = dist
                        best_candidate = (ax, line, y_val)
        
        # 3. Se trovato un candidato valido, crea una NUOVA etichetta
        if best_candidate:
            ax, line, y_val = best_candidate
            annot = ax.annotate(
                f"{line.get_label()}\nT={t_val:.1f}\nVal={y_val:.2f}",
                xy=(t_val, y_val), xytext=(10, 10),
                textcoords="offset points",
                bbox=dict(boxstyle="round", fc="w", alpha=0.9),
                arrowprops=dict(arrowstyle="->"),
                zorder=1000
            )
            all_annotations.append(annot)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', on_click)

    # Gestione chiusura esplicita con tasto 'q'
    def on_key(event):
        if event.key == 'q':
            print("Chiusura grafico...")
            plt.close(fig)
            
    fig.canvas.mpl_connect('key_press_event', on_key)

    print("\n>>> Grafico aperto. Premi 'q' sulla finestra o chiudila per terminare. <<<")
    plt.show()

if __name__ == "__main__":
    main()