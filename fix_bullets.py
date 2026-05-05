import glob

files = glob.glob('/home/ubuntu/Inverter-Design/presentation_v2/*.html')
for f in files:
    with open(f, 'r') as file:
        content = file.read()
    
    # Add explicit list-style to ul and ol
    content = content.replace('ul, ol {', 'ul, ol {\n            list-style-position: outside;\n            margin-left: 40px;\n            padding-left: 20px;')
    content = content.replace('ul {\n            margin-left: 40px;', 'ul {\n            list-style-type: disc;\n            list-style-position: outside;\n            margin-left: 40px;\n            padding-left: 20px;')
    content = content.replace('ol {\n            margin-left: 40px;', 'ol {\n            list-style-type: decimal;\n            list-style-position: outside;\n            margin-left: 40px;\n            padding-left: 20px;')
    
    # If the combined ul, ol { rule exists but doesn't have list-style-type, add it to ul specifically
    if 'ul, ol {' in content and 'list-style-type' not in content:
        content = content.replace('ul, ol {', 'ul { list-style-type: disc; }\n        ol { list-style-type: decimal; }\n        ul, ol {')
        
    with open(f, 'w') as file:
        file.write(content)
print("Bullets fixed")
