import json, sys, urllib.request, urllib.parse, time
UA = {'User-Agent': 'litreview-script/1.0'}
for q in sys.argv[1:]:
    url = 'https://api.crossref.org/works?' + urllib.parse.urlencode({'query.bibliographic': q, 'rows': 3, 'select': 'DOI,title,container-title,issued,author'})
    try:
        j = json.loads(urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=40).read().decode())
    except Exception as e:
        print('ERR', q, e); continue
    print('>>', q)
    for it in j['message']['items']:
        au = '; '.join((a.get('family') or '?') for a in it.get('author', []))[:60]
        yr = ((it.get('issued') or {}).get('date-parts') or [[None]])[0][0]
        print('  ', it['DOI'], '|', yr, '|', (it.get('title') or [''])[0][:90], '|', au, '|', (it.get('container-title') or [''])[0][:40])
