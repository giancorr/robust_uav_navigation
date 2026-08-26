import sys
import os
import shutil
import glob

if len(sys.argv) < 2:
    print("Usage: python3 move_logs.py <subfolder_name>")
    print("Example: python3 move_logs.py tactile")
    sys.exit(1)

subfolder_name = sys.argv[1]

script_dir = os.path.dirname(os.path.abspath(__file__))
dest_dir = os.path.join(script_dir, "..", "old_logs", subfolder_name)
dest_dir = os.path.abspath(dest_dir)

os.makedirs(dest_dir, exist_ok=True)

txt_files = glob.glob(os.path.join(script_dir, "*.txt"))
bt_files = glob.glob(os.path.join(script_dir, "*.bt"))
all_files = txt_files + bt_files

if not all_files:
    print(f"No .txt or .bt files in {script_dir}")
    sys.exit(0)

print(f"Copying {len(all_files)} files to: {dest_dir} ...")

for file_path in all_files:
    filename = os.path.basename(file_path)
    dest_path = os.path.join(dest_dir, filename)
    shutil.copy(file_path, dest_path)
    print(f"  -> Copied: {filename}")
