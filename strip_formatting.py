import os
import glob
from bs4 import BeautifulSoup

slide_files = glob.glob('/home/ubuntu/Inverter-Design/presentation_v2/*.html')

for file in slide_files:
    with open(file, 'r', encoding='utf-8') as f:
        html = f.read()
    
    soup = BeautifulSoup(html, 'html.parser')
    
    # Remove accent bar and course tag
    for el in soup.select('.accent-bar, .course-tag, .divider'):
        el.decompose()
        
    # Simplify styles
    style_tag = soup.find('style')
    if style_tag:
        style_tag.string = """
        * { margin: 0; padding: 0; box-sizing: border-box; }
        .slide-container {
            width: 1280px;
            min-height: 720px;
            background: #FFFFFF;
            position: relative;
            padding: 60px 80px;
            font-family: 'Inter', sans-serif;
            color: #000000;
        }
        h2 {
            font-weight: 700;
            font-size: 36px;
            margin-bottom: 40px;
            border-bottom: 2px solid #000000;
            padding-bottom: 10px;
        }
        p {
            font-size: 20px;
            line-height: 1.5;
            margin-bottom: 20px;
        }
        ul, ol {
            margin-left: 40px;
            margin-bottom: 20px;
            font-size: 20px;
            line-height: 1.5;
        }
        li {
            margin-bottom: 10px;
        }
        .two-col {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 40px;
        }
        table {
            width: 100%;
            border-collapse: collapse;
            margin-bottom: 20px;
            font-size: 18px;
        }
        th, td {
            border: 1px solid #000000;
            padding: 12px;
            text-align: left;
        }
        th {
            background-color: #F0F0F0;
            font-weight: bold;
        }
        .chart-container {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 500px;
        }
        .chart-container img {
            max-width: 100%;
            max-height: 100%;
            object-fit: contain;
        }
        .bold { font-weight: bold; }
        """
        
    # Convert cards and highlight boxes to simple paragraphs or lists
    for card in soup.select('.card, .highlight-box, .spec-box'):
        title = card.find(class_=['card-title', 'title', 'spec-title'])
        title_text = title.get_text(strip=True) if title else ""
        
        text = card.find(class_=['card-text', 'text-content'])
        text_content = text.get_text(strip=True) if text else card.get_text(strip=True).replace(title_text, '')
        
        new_p = soup.new_tag('p')
        if title_text:
            b = soup.new_tag('b')
            b.string = title_text + ": "
            new_p.append(b)
        new_p.append(text_content)
        
        card.replace_with(new_p)
        
    # Remove numbers from cards
    for num in soup.select('.card-num'):
        num.decompose()
        
    # Remove classes that might mess up layout
    for tag in soup.find_all(True):
        if tag.name not in ['div', 'p', 'h2', 'ul', 'li', 'table', 'tr', 'td', 'th', 'img', 'b', 'strong', 'span', 'sup', 'sub', 'html', 'head', 'body', 'style', 'meta', 'link']:
            pass
        if tag.has_attr('class'):
            classes_to_keep = ['slide-container', 'two-col', 'chart-container', 'bold']
            tag['class'] = [c for c in tag['class'] if c in classes_to_keep]
            if not tag['class']:
                del tag['class']
                
    # Specific fix for title slide
    if 'title.html' in file:
        style_tag.string += """
        .title-slide {
            display: flex;
            flex-direction: column;
            justify-content: center;
            height: 100%;
            text-align: center;
        }
        .title-slide h1 {
            font-size: 48px;
            font-weight: bold;
            margin-bottom: 20px;
        }
        .title-slide h3 {
            font-size: 24px;
            font-weight: normal;
            margin-bottom: 40px;
        }
        """
        container = soup.find(class_='slide-container')
        container['class'] = container.get('class', []) + ['title-slide']
        
        # Extract title text
        title_div = soup.find(class_='title')
        if title_div:
            h1 = soup.new_tag('h1')
            h1.string = title_div.get_text()
            title_div.replace_with(h1)
            
        # Extract author text
        author_div = soup.find(class_='author')
        if author_div:
            h3 = soup.new_tag('h3')
            h3.string = author_div.get_text()
            author_div.replace_with(h3)
            
        # Extract affiliation text
        affil_div = soup.find(class_='affiliation')
        if affil_div:
            h3 = soup.new_tag('h3')
            h3.string = affil_div.get_text()
            affil_div.replace_with(h3)

    with open(file, 'w', encoding='utf-8') as f:
        f.write(str(soup))

print("Done stripping formatting.")
