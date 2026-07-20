#!/bin/bash

# 1. Определяем базовую директорию проекта (где лежит этот скрипт)
BASE=$(dirname "$(realpath "$0")")

# 2. Настраиваемый путь к ArduPilot. 
# По умолчанию ищем в ~/ardupilot, но можно переопределить через переменную окружения.
ARDUPILOT_PATH="${ARDUPILOT_PATH:-$HOME/ardupilot}"

# 3. Проверки перед запуском
if [ ! -d "$ARDUPILOT_PATH/Tools/autotest" ]; then
    echo "❌ Ошибка: ArduPilot не найден в $ARDUPILOT_PATH"
    echo "💡 Укажите правильный путь через переменную ARDUPILOT_PATH:"
    echo "   export ARDUPILOT_PATH=/путь/к/вашему/ardupilot"
    exit 1
fi

if ! command -v gz &> /dev/null; then
    echo "❌ Ошибка: Команда 'gz' не найдена. Убедитесь, что новый Gazebo установлен."
    exit 1
fi

# 4. Добавляем локальные модели проекта в путь ресурсов Gazebo
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$BASE/models"

echo "🚁 Проект: $BASE"
echo "🛠  ArduPilot: $ARDUPILOT_PATH"

echo "[1/2] Запуск Gazebo..."
# Запускаем Gazebo в фоне и сохраняем его PID
gz sim -s -v4 -r "$BASE/worlds/my_world.sdf" &
GAZEBO_PID=$!
sleep 5

echo "[2/2] Запуск ArduPilot SITL..."
cd "$ARDUPILOT_PATH"

python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map \
--add-param-file="$BASE/models/iris_with_wings/iris_params.parm" \
--add-param-file="$BASE/joystick.parm" \
--out udp:127.0.0.1:14551

# 5. Корректно убиваем Gazebo, когда ArduPilot (MAVProxy) закрывается
echo "Остановка Gazebo..."
kill $GAZEBO_PID 2>/dev/null