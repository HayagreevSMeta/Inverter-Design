#!/usr/bin/env python3
"""Convert IEEE HTML paper v4 to PDF using WeasyPrint."""
from weasyprint import HTML
import os

os.chdir('/home/ubuntu/Inverter-Design')
html = HTML(filename='IEEE_Paper_Formatted_v4.html', base_url='.')
html.write_pdf('IEEE_Paper_Formatted_v4.pdf')
print("PDF generated: IEEE_Paper_Formatted_v4.pdf")
