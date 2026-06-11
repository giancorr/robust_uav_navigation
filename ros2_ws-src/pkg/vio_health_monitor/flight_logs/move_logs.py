import sys
import os
import shutil
import glob

if len(sys.argv) < 2:
    print("Uso: python3 move_logs.py <nome_sottocartella>")
    print("Esempio: python3 move_logs.py tactile")
    sys.exit(1)

subfolder_name = sys.argv[1]

# Ottieni la directory in cui si trova questo script (flight_logs)
script_dir = os.path.dirname(os.path.abspath(__file__))

# Calcola il percorso della cartella di destinazione (../old_logs/<sottocartella>)
dest_dir = os.path.join(script_dir, "..", "old_logs", subfolder_name)
dest_dir = os.path.abspath(dest_dir)

# Crea la cartella di destinazione se non esiste (crea anche old_logs se necessario)
os.makedirs(dest_dir, exist_ok=True)

# Trova tutti i file .txt nella cartella dello script
txt_files = glob.glob(os.path.join(script_dir, "*.txt"))

if not txt_files:
    print(f"Nessun file .txt trovato in {script_dir}")
    sys.exit(0)

print(f"Copia di {len(txt_files)} file in: {dest_dir} ...")

for file_path in txt_files:
    filename = os.path.basename(file_path)
    dest_path = os.path.join(dest_dir, filename)
    shutil.copy(file_path, dest_path)
    print(f"  -> Copiato: {filename}")

print("\nOperazione completata con successo!")
