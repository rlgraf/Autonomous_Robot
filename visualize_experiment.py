#!/usr/bin/env python3
"""
Visualization script for experiment data.
Creates graphs showing robot trajectory and column visit status.
"""

import os
import csv
import argparse
import matplotlib.pyplot as plt
import numpy as np


def parse_experiment_data(filepath):
    """Parse the consolidated experiment data file."""
    trajectory = []
    columns = {}  # {id: (x, y, visited, visit_time, visit_cycle, after_supervisor)}
    cycles = []
    
    current_section = None
    
    with open(filepath, 'r') as f:
        reader = csv.reader(f, delimiter='\t')
        for row in reader:
            if not row or row[0].startswith('#'):
                if row and row[0].startswith('# Column Locations'):
                    current_section = 'columns'
                elif row and row[0].startswith('# Battery Cycle Summary'):
                    current_section = 'cycles'
                elif row and row[0].startswith('# Robot Trajectory'):
                    current_section = 'trajectory'
                continue
            
            if current_section == 'columns' and len(row) >= 3:
                try:
                    col_id = row[0]
                    x, y = float(row[1]), float(row[2])
                    visited = row[3].lower() == 'true' if len(row) > 3 and row[3] else False
                    visit_time = float(row[4]) if len(row) > 4 and row[4] else None
                    visit_cycle = int(row[5]) if len(row) > 5 and row[5] else None
                    after_supervisor = row[6].lower() == 'true' if len(row) > 6 and row[6] else False
                    columns[col_id] = (x, y, visited, visit_time, visit_cycle, after_supervisor)
                except (ValueError, IndexError):
                    continue
            
            elif current_section == 'cycles' and len(row) >= 4:
                try:
                    cycles.append({
                        'cycle_id': int(row[0]),
                        'cycle_start_time': float(row[1]),
                        'charge_start_time': float(row[2]),
                        'columns_this_cycle': int(row[3]),
                        'columns_after_25': int(row[4]) if len(row) > 4 else 0
                    })
                except (ValueError, IndexError):
                    continue
            
            elif current_section == 'trajectory' and len(row) >= 3:
                try:
                    trajectory.append([float(row[0]), float(row[1]), float(row[2])])
                except (ValueError, IndexError):
                    continue
    
    return trajectory, columns, cycles


def plot_experiment(trajectory, columns, cycles, output_file=None):
    """Create visualization of robot path and column visits."""
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Plot trajectory
    if trajectory:
        traj_array = np.array(trajectory)
        ax.plot(traj_array[:, 1], traj_array[:, 2], 'b-', linewidth=1.5, alpha=0.6, label='Robot Path')
        # Mark start
        if len(traj_array) > 0:
            ax.plot(traj_array[0, 1], traj_array[0, 2], 'go', markersize=10, label='Start')
        # Mark end
        if len(traj_array) > 0:
            ax.plot(traj_array[-1, 1], traj_array[-1, 2], 'ro', markersize=10, label='End')
    
    # Plot columns
    visited_cols = []
    unvisited_cols = []
    supervisor_visited = []
    
    for col_id, (x, y, visited, visit_time, visit_cycle, after_supervisor) in columns.items():
        if visited:
            if after_supervisor:
                supervisor_visited.append((x, y))
            else:
                visited_cols.append((x, y))
        else:
            unvisited_cols.append((x, y))
    
    # Plot unvisited columns (red)
    if unvisited_cols:
        uv_array = np.array(unvisited_cols)
        ax.scatter(uv_array[:, 0], uv_array[:, 1], c='red', s=150, marker='o', 
                  edgecolors='darkred', linewidths=2, label='Unvisited Columns', zorder=5)
    
    # Plot visited columns (green)
    if visited_cols:
        v_array = np.array(visited_cols)
        ax.scatter(v_array[:, 0], v_array[:, 1], c='green', s=150, marker='s', 
                  edgecolors='darkgreen', linewidths=2, label='Visited Columns', zorder=5)
    
    # Plot supervisor-visited columns (orange)
    if supervisor_visited:
        sv_array = np.array(supervisor_visited)
        ax.scatter(sv_array[:, 0], sv_array[:, 1], c='orange', s=150, marker='^', 
                  edgecolors='darkorange', linewidths=2, label='Visited After Supervisor', zorder=5)
    
    # Add column labels
    for col_id, (x, y, visited, visit_time, visit_cycle, after_supervisor) in columns.items():
        ax.annotate(col_id.replace('column_', ''), (x, y), fontsize=8, 
                   ha='center', va='bottom', alpha=0.7)
    
    # Plot charging stations
    charging_stations = [(-20.0, 0.0), (20.0, 0.0)]
    for i, (cx, cy) in enumerate(charging_stations):
        label = 'Charging Station' if i == 0 else None
        ax.scatter(cx, cy, c='blue', s=300, marker='*', 
                  edgecolors='darkblue', linewidths=2, label=label, zorder=6)
    
    # Formatting
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Robot Trajectory and Column Visit Status', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)
    ax.set_aspect('equal', adjustable='box')
    
    # Add statistics text
    stats_text = (
        f"Total Columns: {len(columns)}\n"
        f"Visited: {len(visited_cols) + len(supervisor_visited)}\n"
        f"Unvisited: {len(unvisited_cols)}\n"
        f"After Supervisor: {len(supervisor_visited)}\n"
        f"Battery Cycles: {len(cycles)}"
    )
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
           fontsize=10, verticalalignment='top', 
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Graph saved to {output_file}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize experiment data')
    parser.add_argument('data_file', type=str, help='Path to experiment_data.tsv file')
    parser.add_argument('-o', '--output', type=str, help='Output image file (e.g., plot.png)')
    parser.add_argument('--show', action='store_true', help='Display plot interactively')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.data_file):
        print(f"Error: File not found: {args.data_file}")
        return
    
    print(f"Parsing data from {args.data_file}...")
    trajectory, columns, cycles = parse_experiment_data(args.data_file)
    
    print(f"Found {len(trajectory)} trajectory points")
    print(f"Found {len(columns)} columns")
    print(f"Found {len(cycles)} battery cycles")
    
    plot_experiment(trajectory, columns, cycles, output_file=args.output)
    
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
