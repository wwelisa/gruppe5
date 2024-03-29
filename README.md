# SOA Projekt

## Achtung!!

Um in der map_hexa.yaml file muss der absolute Pfad von der map_hexa.pgm Datei angegeben werden. Dies kann von System zu System variieren.

# Ausführen

Zuerst catkin_make im catkin workspace. Zum Ausführen in den scripts Ordner gehen und dort das startPrograms script ausführen.

## Git installieren

[Downloaden](https://git-scm.com/downloads) und installieren.

Nach Abschluss der Installation sind "git" Befehele über die Konsole ausführbar.

## Git workflow

- Git Repository klonen: 
  ```bash
  git clone https://github.com/wwelisa/gruppe5.git
  ```
- Falls das Repo lokal schon verfügbar ist aber eventuell nicht aktuell ist mit git pull updaten.
  ```bash
  git pull
  ```
  Repo ist nun im Ordner wo der clone Befehl ausgeführt wurde lokal verfügbar bzw. auf aktuellem Stand.

- Jetzt kann am Projekt gearbeitet werden. Allerdings ist es ratsam einen neuen Branch zu erstellen falls ein neues Feature zu bereits funktionierendem Code hinzugefügt wird.
  ```bash
  git checkout -b new-feature
  ```
  Dadurch wird ein Branch namens "new-feature" auf Basis des main-Branches ausgecheckt und das Flag -b weist Git an, den Branch zu erstellen, sofern es ihn noch nicht gibt.

- Sobald Fortschritte erzielt wurden sollten diese commited werden. Dazu alle Änderungen "stagen" (bestimmt quasi welche Änderungen commited werden).
  mit 
  ```bash 
  git add . 
  ``` 
  können alle Änderungen gestaged werden. 
  Danach können mit
  ```bash 
  git commit -m "Sinnvolle Commitnachricht"
  ```
  alle gestageden Änderungen Commited werden.

- Alles was bisher commited wurde ist bis jetzt nur im lokalen Verzeichnis verfügbar. Damit die Änderungen auch remote verfügbar sind folgenden Befehl ausführen.
  ```bash
  git push
  ```
### Einen Branch in den main Branch mergen

Dazu wird in der GitLab Weboberfläche ein Mergerequest erstellt. Idealerweise sollte ein Branch nicht von dem jenigen der das neue Feature implementiert hat gemered werden.
Um einen Mergrequest zu erstellen folge dieser [Anleitung](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html).

## Ausführliche Anleitung zu Git
Für eine ausführliche Anleitung zu Git klicke [hier](https://docs.gitlab.com/ee/gitlab-basics/start-using-git.html#create-a-branch).


  
