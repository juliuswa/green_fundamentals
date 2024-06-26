# Things to change

- Localization 
    - Wenn er lost ist, dann vielleicht in jeder Zelle neue Partikel spawnen und schauen ob es dort möglich ist und zu den scans passt.
    - Grid Localization als alternative
    - AMCL lokalisiert nur, wenn genug movement kam?
    - AMCL Code orientieren

- Path planning
    - Verändere die Breitensuche, sodass er auch checkt ob man diagonal fahren kann. Wahrscheinlich in get_neighbors wo er checken muss, ob es möglich ist direkt in die diagonalen zellen zu fahren