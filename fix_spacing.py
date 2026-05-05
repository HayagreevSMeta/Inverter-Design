import glob
import os

files = glob.glob('/home/ubuntu/Inverter-Design/presentation_v2/*.html')
for f in files:
    with open(f, 'r') as file:
        content = file.read()
    
    # Add vertical distribution flex to slide-container
    content = content.replace('.slide-container {', '.slide-container {\n            display: flex;\n            flex-direction: column;')
    content = content.replace('h2 {', 'h2 {\n            flex: 0 0 auto;')
    content = content.replace('.two-col {', '.two-col {\n            flex: 1 1 auto;\n            align-items: center;')
    
    with open(f, 'w') as file:
        file.write(content)
print("Spacing fixed")
