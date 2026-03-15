#!/usr/bin/env python3
"""
Wrapper script for arena2.py that handles optional coordinates_file argument.
This allows the launch file to always pass --coordinates-file even if empty.
"""

import sys
import subprocess
from pathlib import Path

def main():
    # Get the path to arena2.py (should be in the same directory)
    script_dir = Path(__file__).parent
    arena2_script = script_dir / "arena2.py"
    
    if not arena2_script.exists():
        print(f"Error: arena2.py not found at {arena2_script}")
        sys.exit(1)
    
    # Build command - filter out empty coordinates_file arguments
    cmd = ["python3", str(arena2_script)]
    args = sys.argv[1:]
    
    i = 0
    while i < len(args):
        if args[i] == "--coordinates-file" and i + 1 < len(args):
            # Check if the value is non-empty
            if args[i + 1].strip():
                cmd.extend([args[i], args[i + 1]])
                i += 2
            else:
                # Skip empty coordinates_file argument
                i += 2
        else:
            cmd.append(args[i])
            i += 1
    
    # Execute arena2.py with filtered arguments
    sys.exit(subprocess.call(cmd))

if __name__ == "__main__":
    main()
