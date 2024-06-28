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