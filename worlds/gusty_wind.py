#!/usr/bin/env python3
"""
Спам силы через ApplyLinkWrench.
Обходит баги WindEffects и блокировку ArduPilot.
"""
import subprocess
import time
import os

# ID дрона из твоих логов. Если после перезапуска Gazebo изменится - поменяй тут.
# (Можно узнать командой: gz topic -e -t /world/iris_runway/state | grep -B 1 "iris_with_wings")
ENTITY_ID = 15
TOPIC = "/world/iris_runway/wrench"

print(f"💥 Запуск спама силы для entity {ENTITY_ID}...")
print("⚠️  Убедись, что дрон в воздухе (ACRO)!")
print("Нажми Ctrl+C для остановки.\n")

# Формируем сообщение один раз, чтобы ускорить цикл
# Сила 800 Ньютонов (достаточно, чтобы сдуть 1.5кг дрон)
msg = f"entity {{ id: {ENTITY_ID} }} wrench {{ force {{ x: 800 y: 0 z: 0 }} }}"

cmd = [
    "gz", "topic",
    "-t", TOPIC,
    "-m", "gz.msgs.EntityWrench",
    "-p", msg
]

try:
    while True:
        # Спамим команду. subprocess.run быстр, но не идеален.
        # Для максимальной скорости можно использовать os.system, но subprocess надежнее.
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # Небольшая задержка, чтобы не повесить CPU (50-100 Гц)
        time.sleep(0.01) 

except KeyboardInterrupt:
    print("\n🛑 Остановка спама...")
    # Отправляем нулевую силу, чтобы отпустить дрон
    zero_msg = f"entity {{ id: {ENTITY_ID} }} wrench {{ force {{ x: 0 y: 0 z: 0 }} }}"
    subprocess.run([
        "gz", "topic", "-t", TOPIC, "-m", "gz.msgs.EntityWrench", "-p", zero_msg
    ])
    print("✅ Готово")