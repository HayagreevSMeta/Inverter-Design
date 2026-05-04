#!/usr/bin/env python3
"""Convert IEEE HTML paper v2 to PDF using WeasyPrint."""
from weasyprint import HTML
import os

os.chdir('/home/ubuntu/Inverter-Design')
html = HTML(filename='IEEE_Paper_Formatted_v2.html', base_url='.')
html.write_pdf('IEEE_Paper_Formatted_v2.pdf')
print("PDF generated: IEEE_Paper_Formatted_v2.pdf")
