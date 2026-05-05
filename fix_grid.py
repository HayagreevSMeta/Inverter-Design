import glob
import os

files = glob.glob('/home/ubuntu/Inverter-Design/presentation_v2/*.html')
for f in files:
    with open(f, 'r') as file:
        content = file.read()
    
    # Fix the standard two-col layout to be strictly 50/50 and ensure images don't stretch it
    content = content.replace('grid-template-columns: 1fr 1fr;', 'grid-template-columns: 1fr 1fr;\n            min-width: 0;')
    
    # Fix the two-col-wide-right layout (used in vehicle mapping) to give text more space
    content = content.replace('grid-template-columns: 1fr 2fr;', 'grid-template-columns: 1.5fr 1fr;\n            min-width: 0;')
    
    # Ensure chart-container has a fixed max width so it doesn't force the grid column to expand
    content = content.replace('.chart-container {', '.chart-container {\n            max-width: 100%;\n            min-width: 0;')
    
    with open(f, 'w') as file:
        file.write(content)
print("Grid fixed")
