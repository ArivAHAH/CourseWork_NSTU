# EVA — Симуляция трафика (курсовой проект)

Qt (Widgets + WebEngine) приложение + PostgreSQL (Docker).

---

## Требования
- Docker
- Docker Compose

---

## Подготовка (один раз)

В корне проекта:

```bash
cp .env.example .env

#Linux
###1. Разрешить Docker доступ к X11
xhost +local:docker

###2. Запуск приложения
docker compose -p traffic_sim --profile linux-gui up --build

###3. Остановка
docker compose -p traffic_sim down

###4. Полная очистка (включая БД)
docker compose -p traffic_sim down -v


## Требования для macOS

Для запуска графического приложения локально необходимо:
- Qt 6 (Widgets, WebEngine)
- CMake

Для базы данных используется Docker (PostgreSQL внутри контейнера),
локальная установка PostgreSQL не требуется.
# macOS (Docker — только БД, приложение локально)
### Установка зависимостей (macOS, через Homebrew)
```bash
brew install cmake qt

### 1. Запуск базы данных
```bash
docker compose -p traffic_sim up -d db

###2. Сборка проекта
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

###3. Передача переменных окружения
export DB_HOST=localhost
export DB_PORT=5432
export DB_NAME=$(grep DB_NAME .env | cut -d= -f2)
export DB_USER=$(grep DB_USER .env | cut -d= -f2)
export DB_PASS=$(grep DB_PASS .env | cut -d= -f2)

###4. Запуск приложения
./build/traffic-sim

###5. Остановка базы данных
docker compose -p traffic_sim down
