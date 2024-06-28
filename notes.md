# Things to change
- Zwei Phasen
    1. Globale Lokalisierung
        - viele Partikel ~5000
        - solange bis auf einen Punkt converged
        - fahre langsamer und in möglichst ruckelfreien routen
        - Wenn converged -> umschalten auf lokale lokalisierung
        - ??? Wie merken, dass converged: Merkt das der Lokaliser oder der Planner
    2. Lokale Lokalisierung: wenn ich ein bisschen lost bin, dann sample neue samples in der umgebung, um dich wieder zu finden.
        - Weniger Partikel
        - Sample bei unsicherheit nur lokal um die letzte Position (vllt mit steigender Varianz, wenn es garnicht klappt)
        - Es gibt kein Kidnapping
        - **TODO** wenn er irgendwo hinfahren will und er würde gegen eine Wand fahren, dann kann er globalisieren

# Globalizer
- Besseres Sampling mit weniger Varianz
- Merke, wenn es nicht funktioniert hat, bevor man has_converged ausgibt und starte neu
    - Schaue auf die Entwicklung der weights und wenn es zu schlecht ist, dann starte einfach neu.