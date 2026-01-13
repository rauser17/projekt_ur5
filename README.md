# Projekt Sterowania Manipulatorem UR5

## Autorzy
* **Bartosz Dworazny**
* **Wojciech Czarnowski**

## Wymagania
Do poprawnego działania projektu wymagane są:
* System operacyjny Linux (zalecane Ubuntu 24.04)
* Docker
* Git

## Środowisko
Projekt został przygotowany w oparciu o kontenerystykę, co zapewnia spójność środowiska:
* **System operacyjny:** Ubuntu 24.04 LTS (Noble Numbat)
* **Middleware:** ROS 2 Jazzy

## Sklonowanie projektu
   ```bash
   git clone [https://github.com/rauser17/projekt_ur5.git](https://github.com/rauser17/projekt_ur5.git)
   cd projekt_ur5# Projekt UR5

## Pobranie i uruchomienie projektu
1. sklonowanie repozytorium na dysk;
2. nadanie uprawnien do wykonania skrytpu startowego oraz zezwolenie na wyswietlanie okien z Dockera: `chmod +x start_docker.sh`, `xhost +`;
3. uruchomienie skryptu (budowa+start): `./start_docker.sh`;

## Opis elementów zrealizowanych w projekcie:
1. **Logika sterowania**
    * Kliknięcie powyżej środka: Ruch ramienia w górę.
    * Kliknięcie poniżej środka: Ruch ramienia w dół.
2.  **Automatyzacj**
    * Plik `ur5_projekt.launch.py` uruchamia jednym poleceniem: symulację UR5, RViz2, węzeł kamery i kontroler.
3.  **Robot UR5**
    * Użycie profesjonalnego manipulatora UR5 zamiast domyślnego TurtleBota.
4.  **Dockeryzacja** 
    * Kompletny setup. Obraz bazuje na `ros:jazzy-ros-base` (Ubuntu 24.04 Noble Numbat).
