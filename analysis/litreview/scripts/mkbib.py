import json, os, sys, re, html, urllib.request, urllib.parse
D = sys.argv[1]
UA = {'User-Agent': 'litreview-script/1.0'}
YEAR_FIX = {'moravec1985high': 1985}
def clean(s): 
    s = html.unescape(re.sub(r'<[^>]+>', '', s or '')).replace('‐', '-').replace('–', '--').replace('—', '--')
    return re.sub(r'\s+', ' ', s).strip()
def yr(m):
    for k in ('published-print', 'issued', 'published-online'):
        dp = (m.get(k) or {}).get('date-parts') or [[None]]
        if dp[0][0]: return dp[0][0]
inc = [l.split() for l in open(os.path.join(D, 'scripts', 'included.txt')) if l.strip()]
bib = []; table = []
for t, key, doi in inc:
    fn = os.path.join(D, 'verify', doi.lower().replace('/', '_').replace('(', '').replace(')', '') + '.json')
    if not os.path.exists(fn):
        fn2 = os.path.join(D, 'verify', doi.replace('/', '_').replace('(', '').replace(')', '') + '.json')
        fn = fn2 if os.path.exists(fn2) else fn
    if os.path.exists(fn): m = json.load(open(fn))['message']
    else:
        m = json.loads(urllib.request.urlopen(urllib.request.Request('https://api.crossref.org/works/' + urllib.parse.quote(doi, safe='/'), headers=UA), timeout=40).read().decode())['message']
        json.dump({'message': m}, open(fn, 'w'))
    authors = m.get('author') or []
    oa_auth = {'10.1109/37.887448': ['Stilwell, Daniel J.', 'Bishop, Bradley E.'], '10.1109/mcs.2009.934469': ['Bar-Shalom, Yaakov', 'Daum, Fred E.', 'Huang, Jim']}
    if authors: au = ' and '.join(f"{clean(a.get('family'))}, {clean(a.get('given'))}" if a.get('given') else clean(a.get('family') or a.get('name')) for a in authors)
    else: au = ' and '.join(oa_auth.get(doi.lower(), ['MISSING']))
    y = YEAR_FIX.get(key) or yr(m)
    ty = m.get('type'); ct = clean((m.get('container-title') or [''])[0]); title = clean((m.get('title') or [''])[0])
    f = {'author': au, 'title': '{' + title + '}', 'year': str(y), 'doi': m['DOI']}
    if ty == 'journal-article':
        et = 'article'; f['journal'] = ct
        if m.get('volume'): f['volume'] = m['volume']
        if m.get('issue'): f['number'] = m['issue']
    elif ty == 'proceedings-article': et = 'inproceedings'; f['booktitle'] = ct
    elif ty == 'book-chapter':
        et = 'incollection'; f['booktitle'] = ct
        if m.get('publisher'): f['publisher'] = m['publisher']
    else:
        et = 'book'; f['publisher'] = m.get('publisher', '')
    if m.get('page'): f['pages'] = m['page'].replace('-', '--')
    if m.get('article-number') and 'pages' not in f: f['pages'] = m['article-number']
    bib.append(f'@{et}{{{key},\n' + ',\n'.join(f'  {k} = {{{v}}}' if k != 'title' else f'  {k} = {{{v}}}' for k, v in f.items()) + '\n}')
    first = clean(authors[0].get('family')) if authors else (oa_auth.get(doi.lower(), ['?'])[0].split(',')[0])
    etal = ' et al.' if len(authors) > 2 else (' and ' + clean(authors[1].get('family')) if len(authors) == 2 else '')
    if not authors and doi.lower() in oa_auth: etal = ' et al.' if len(oa_auth[doi.lower()]) > 2 else ' and ' + oa_auth[doi.lower()][1].split(',')[0]
    vol = (m.get('volume') or ''); iss = m.get('issue') or ''; pg = (m.get('page') or m.get('article-number') or '')
    loc = (f', {vol}' + (f'({iss})' if iss else '') if vol else '') + (f', {pg}' if pg else '')
    table.append('\t'.join([t, key, f'{first}{etal} ({y}). {title}. *{ct or m.get("publisher","")}*{loc}.', m['DOI']]))
open(os.path.join(D, 'included.bib'), 'w', encoding='utf-8').write('\n\n'.join(bib) + '\n')
open(os.path.join(D, 'included_table.tsv'), 'w', encoding='utf-8').write('\n'.join(table) + '\n')
print(len(bib), 'entries'); print(open(os.path.join(D, 'included_table.tsv'), encoding='utf-8').read())
