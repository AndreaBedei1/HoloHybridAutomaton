import json, sys, os, re, urllib.request, urllib.parse, time
D = sys.argv[1]; dois = sys.argv[2:]
UA = {'User-Agent': 'litreview-script/1.0'}
os.makedirs(os.path.join(D, 'abstracts'), exist_ok=True)
for doi in dois:
    fn = os.path.join(D, 'verify', doi.replace('/', '_') + '.json')
    ab = None; src = ''
    if os.path.exists(fn):
        m = json.load(open(fn))['message']
        if m.get('abstract'): ab = re.sub('<[^>]+>', ' ', m['abstract']); src = 'CR'
    if not ab:
        afn = os.path.join(D, 'abstracts', doi.replace('/', '_') + '.json')
        if os.path.exists(afn): j = json.load(open(afn))
        else:
            try:
                j = json.loads(urllib.request.urlopen(urllib.request.Request('https://api.openalex.org/works/doi:' + doi + '?select=title,abstract_inverted_index,authorships', headers=UA), timeout=40).read().decode())
                json.dump(j, open(afn, 'w'))
            except Exception as e: j = {}; print('ERR', doi, e)
        inv = j.get('abstract_inverted_index')
        if inv:
            pos = sorted((p, w) for w, ps in inv.items() for p in ps); ab = ' '.join(w for _, w in pos); src = 'OA'
        if j.get('authorships') is not None and doi in ('10.1109/37.887448', '10.1109/mcs.2009.934469', '10.1109/mcs.2008.929280'):
            print('AUTHORS', doi, [a['author']['display_name'] for a in j['authorships']])
    print(f'== {doi} [{src}] ' + (re.sub(r'\s+', ' ', ab)[int(os.environ.get("ABN","650")) if False else 650:1500] if ab else 'NO ABSTRACT'))
