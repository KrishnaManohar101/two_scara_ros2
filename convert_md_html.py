
import sys
import os
import re

# MathJax configuration for inline ($...$) and display ($$...$$) math
MATHJAX_HEADER = """
<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
<script>
  window.MathJax = {
    tex: {
      inlineMath: [['$', '$'], ['\\\\(', '\\\\)']],
      displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']],
      processEscapes: true
    }
  };
</script>
<style>
body { font-family: sans-serif; line-height: 1.6; padding: 20px; max-width: 800px; margin: 0 auto; }
h1, h2, h3 { color: #333; }
code { background: #f4f4f4; padding: 2px 5px; border-radius: 3px; }
pre { background: #f4f4f4; padding: 10px; border-radius: 5px; overflow-x: auto; }
</style>
"""

def simple_markdown_parser(text):
    html = []
    lines = text.split('\n')
    in_code_block = False
    
    for line in lines:
        # Code blocks
        if line.strip().startswith('```'):
            in_code_block = not in_code_block
            if in_code_block:
                html.append('<pre><code>')
            else:
                html.append('</code></pre>')
            continue
            
        if in_code_block:
            html.append(line + '\n')
            continue

        # Headers
        if line.startswith('# '):
            html.append(f'<h1>{line[2:]}</h1>')
        elif line.startswith('## '):
            html.append(f'<h2>{line[3:]}</h2>')
        elif line.startswith('### '):
            html.append(f'<h3>{line[4:]}</h3>')
        elif line.startswith('#### '):
            html.append(f'<h4>{line[5:]}</h4>')
        # Horizontal Rule
        elif line.strip() == '---':
            html.append('<hr>')
        # Lists (Basic)
        elif line.strip().startswith('* ') or line.strip().startswith('- '):
            html.append(f'<li>{line.strip()[2:]}</li>')
        # Empty lines
        elif line.strip() == '':
            html.append('<br>')
        # Paragraphs
        else:
            html.append(f'<p>{line}</p>')
            
    return '\n'.join(html)

def convert(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        text = f.read()

    html_content = ""
    try:
        import markdown
        # Convert using library if available
        html_body = markdown.markdown(text, extensions=['extra', 'codehilite'])
        html_content = html_body
    except Exception as e:
        print(f"Markdown library failed ({e}), using simple parser.")
        html_content = simple_markdown_parser(text)

    # Wrap in full HTML
    full_html = f"""
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Document</title>
{MATHJAX_HEADER}
</head>
<body>
{html_content}
</body>
</html>
    """
    
    output_filename = os.path.splitext(filename)[0] + ".html"
    with open(output_filename, 'w', encoding='utf-8') as f:
        f.write(full_html)
    
    print(f"Successfully created {output_filename}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 convert_md_html.py <file.md>")
        sys.exit(1)
    
    convert(sys.argv[1])
