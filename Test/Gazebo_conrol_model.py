import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import json
import os
import re

class GazeboController:
    def __init__(self, root):
        self.root = root
        self.root.title("Gazebo Model Controller")
        self.root.geometry("900x700")
        
        # Нейтральный фон
        self.root.configure(bg='#f0f0f0')
        
        # Файл для сохранения моделей
        self.config_file = "gazebo_models.json"
        
        # Загружаем сохранённые модели и их исходные позиции
        self.models_data = self.load_models()
        
        # Словарь для хранения виджетов каждой модели
        self.model_widgets = {}
        
        # Имя мира (нужно для некоторых команд)
        self.world_name = "default"  # Измени на имя твоего мира
        
        # Создаём интерфейс
        self.create_interface()
        
        # Отображаем сохранённые модели
        for model_name, model_data in self.models_data.items():
            self.add_model_widget(model_name, model_data)
    
    def create_interface(self):
        # Заголовок
        title_label = tk.Label(
            self.root, 
            text="Gazebo Model Controller", 
            font=("Arial", 16, "bold"),
            bg='#f0f0f0'
        )
        title_label.pack(pady=10)
        
        # Панель добавления модели
        add_frame = tk.Frame(self.root, bg='#f0f0f0')
        add_frame.pack(pady=10, padx=20, fill='x')
        
        tk.Label(add_frame, text="Имя модели:", bg='#f0f0f0', font=("Arial", 10)).pack(side='left', padx=5)
        
        self.model_entry = tk.Entry(add_frame, width=30, font=("Arial", 10))
        self.model_entry.pack(side='left', padx=5)
        self.model_entry.bind('<Return>', lambda e: self.add_model())
        
        add_button = tk.Button(
            add_frame, 
            text="Добавить модель", 
            command=self.add_model,
            bg='#4CAF50',
            fg='white',
            font=("Arial", 10, "bold"),
            padx=10
        )
        add_button.pack(side='left', padx=5)
        
        # Разделитель
        separator = ttk.Separator(self.root, orient='horizontal')
        separator.pack(fill='x', padx=20, pady=10)
        
        # Область для моделей (с прокруткой)
        self.models_frame = tk.Frame(self.root, bg='#f0f0f0')
        self.models_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        # Создаём Canvas для прокрутки
        self.canvas = tk.Canvas(self.models_frame, bg='#f0f0f0', highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self.models_frame, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = tk.Frame(self.canvas, bg='#f0f0f0')
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
    
    def get_model_pose(self, model_name):
        """Получить текущую позицию модели"""
        try:
            # Команда для получения позиции модели
            cmd = f'gz topic -t "/world/{self.world_name}/pose/info" --echo --duration 1'
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=2)
            
            # Парсим вывод
            output = result.stdout
            
            # Ищем позицию нужной модели
            pattern = rf'name: "{model_name}".*?position.*?x: ([\d\.-]+).*?y: ([\d\.-]+).*?z: ([\d\.-]+).*?orientation.*?x: ([\d\.-]+).*?y: ([\d\.-]+).*?z: ([\d\.-]+).*?w: ([\d\.-]+)'
            
            match = re.search(pattern, output, re.DOTALL)
            
            if match:
                pose = {
                    'x': float(match.group(1)),
                    'y': float(match.group(2)),
                    'z': float(match.group(3)),
                    'qx': float(match.group(4)),
                    'qy': float(match.group(5)),
                    'qz': float(match.group(6)),
                    'qw': float(match.group(7))
                }
                return pose
            else:
                # Если не нашли, возвращаем нулевую позицию
                return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
                
        except Exception as e:
            print(f"Ошибка получения позиции: {e}")
            # Возвращаем нулевую позицию по умолчанию
            return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    
    def add_model(self):
        model_name = self.model_entry.get().strip()
        
        if not model_name:
            messagebox.showwarning("Ошибка", "Введите имя модели!")
            return
        
        if model_name in self.models_data:
            messagebox.showwarning("Ошибка", "Модель с таким именем уже добавлена!")
            return
        
        # Получаем текущую позицию модели
        initial_pose = self.get_model_pose(model_name)
        
        # Добавляем модель с её начальной позицией
        self.models_data[model_name] = {
            'initial_pose': initial_pose
        }
        self.save_models()
        
        # Создаём виджеты для модели
        self.add_model_widget(model_name, self.models_data[model_name])
        
        # Очищаем поле ввода
        self.model_entry.delete(0, tk.END)
        
        messagebox.showinfo("Успех", f"Модель '{model_name}' добавлена!\nНачальная позиция сохранена.")
    
    def add_model_widget(self, model_name, model_data):
        # Рамка для модели
        model_frame = tk.LabelFrame(
            self.scrollable_frame, 
            text=f"{model_name} (Начальная: x={model_data['initial_pose']['x']:.2f}, y={model_data['initial_pose']['y']:.2f}, z={model_data['initial_pose']['z']:.2f})", 
            font=("Arial", 10, "bold"),
            bg='#e8e8e8',
            padx=10,
            pady=10
        )
        model_frame.pack(fill='x', pady=5, padx=5)
        
        # Словарь для хранения виджетов этой модели
        widgets = {}
        
        # Поля для скоростей
        speeds_frame = tk.Frame(model_frame, bg='#e8e8e8')
        speeds_frame.pack(fill='x', pady=5)
        
        # Скорость X
        tk.Label(speeds_frame, text="Скорость X:", bg='#e8e8e8').grid(row=0, column=0, padx=5, pady=2, sticky='w')
        widgets['vx'] = tk.Entry(speeds_frame, width=10)
        widgets['vx'].grid(row=0, column=1, padx=5, pady=2)
        widgets['vx'].insert(0, "0.0")
        
        # Скорость Y
        tk.Label(speeds_frame, text="Скорость Y:", bg='#e8e8e8').grid(row=0, column=2, padx=5, pady=2, sticky='w')
        widgets['vy'] = tk.Entry(speeds_frame, width=10)
        widgets['vy'].grid(row=0, column=3, padx=5, pady=2)
        widgets['vy'].insert(0, "0.0")
        
        # Скорость Z
        tk.Label(speeds_frame, text="Скорость Z:", bg='#e8e8e8').grid(row=0, column=4, padx=5, pady=2, sticky='w')
        widgets['vz'] = tk.Entry(speeds_frame, width=10)
        widgets['vz'].grid(row=0, column=5, padx=5, pady=2)
        widgets['vz'].insert(0, "0.0")
        
        # Угловая скорость
        tk.Label(speeds_frame, text="Угловая скорость:", bg='#e8e8e8').grid(row=0, column=6, padx=5, pady=2, sticky='w')
        widgets['angular'] = tk.Entry(speeds_frame, width=10)
        widgets['angular'].grid(row=0, column=7, padx=5, pady=2)
        widgets['angular'].insert(0, "0.0")
        
        # Кнопки управления
        buttons_frame = tk.Frame(model_frame, bg='#e8e8e8')
        buttons_frame.pack(fill='x', pady=5)
        
        send_button = tk.Button(
            buttons_frame,
            text="Отправить команду",
            command=lambda m=model_name, w=widgets: self.send_command(m, w),
            bg='#2196F3',
            fg='white',
            font=("Arial", 10, "bold"),
            padx=10
        )
        send_button.pack(side='left', padx=5)
        
        reset_button = tk.Button(
            buttons_frame,
            text="Вернуть в исходное",
            command=lambda m=model_name, d=model_data: self.reset_to_initial(m, d),
            bg='#FF9800',
            fg='white',
            font=("Arial", 10, "bold"),
            padx=10
        )
        reset_button.pack(side='left', padx=5)
        
        update_pose_button = tk.Button(
            buttons_frame,
            text="Обновить начальную позицию",
            command=lambda m=model_name, f=model_frame, d=model_data: self.update_initial_pose(m, f, d),
            bg='#9C27B0',
            fg='white',
            font=("Arial", 10),
            padx=10
        )
        update_pose_button.pack(side='left', padx=5)
        
        delete_button = tk.Button(
            buttons_frame,
            text="Удалить модель",
            command=lambda m=model_name, f=model_frame: self.delete_model(m, f),
            bg='#f44336',
            fg='white',
            font=("Arial", 10),
            padx=10
        )
        delete_button.pack(side='left', padx=5)
        
        # Сохраняем виджеты
        self.model_widgets[model_name] = widgets
    
    def send_command(self, model_name, widgets):
        try:
            # Получаем значения из полей
            vx = float(widgets['vx'].get())
            vy = float(widgets['vy'].get())
            vz = float(widgets['vz'].get())
            angular = float(widgets['angular'].get())
            
            # Формируем команду
            cmd = f'gz topic -t "/model/{model_name}/cmd_vel" -m gz.msgs.Twist -p "linear: {{x: {vx}, y: {vy}, z: {vz}}}, angular: {{z: {angular}}}"'
            
            # Выполняем команду
            subprocess.run(cmd, shell=True, check=True)
            
            print(f"Команда отправлена для модели {model_name}: vx={vx}, vy={vy}, vz={vz}, angular={angular}")
            
        except ValueError:
            messagebox.showerror("Ошибка", "Введите корректные числовые значения!")
        except subprocess.CalledProcessError as e:
            messagebox.showerror("Ошибка", f"Ошибка выполнения команды:\n{e}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Непредвиденная ошибка:\n{e}")
    
    def reset_to_initial(self, model_name, model_data):
        """Вернуть модель в исходное положение"""
        try:
            pose = model_data['initial_pose']
            
            # Команда для установки позиции модели
            cmd = f'''gz service -s "/world/{self.world_name}/set_pose" \
--reqtype gz.msgs.Pose --resptype gz.msgs.Boolean \
--timeout 5000 \
--req "name: '{model_name}', position: {{x: {pose['x']}, y: {pose['y']}, z: {pose['z']}}}, orientation: {{x: {pose['qx']}, y: {pose['qy']}, z: {pose['qz']}, w: {pose['qw']}}}"'''
            
            subprocess.run(cmd, shell=True, check=True)
            
            messagebox.showinfo("Успех", f"Модель '{model_name}' возвращена в исходное положение!")
            
        except subprocess.CalledProcessError as e:
            messagebox.showerror("Ошибка", f"Ошибка установки позиции:\n{e}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Непредвиденная ошибка:\n{e}")
    
    def update_initial_pose(self, model_name, frame, model_data):
        """Обновить начальную позицию модели"""
        if messagebox.askyesno("Подтверждение", f"Обновить начальную позицию для '{model_name}'?"):
            new_pose = self.get_model_pose(model_name)
            model_data['initial_pose'] = new_pose
            self.save_models()
            
            # Обновляем заголовок рамки
            frame.config(text=f"{model_name} (Начальная: x={new_pose['x']:.2f}, y={new_pose['y']:.2f}, z={new_pose['z']:.2f})")
            
            messagebox.showinfo("Успех", f"Начальная позиция обновлена!")
    
    def delete_model(self, model_name, frame):
        if messagebox.askyesno("Подтверждение", f"Удалить модель '{model_name}'?"):
            # Удаляем из словаря
            if model_name in self.models_data:
                del self.models_data[model_name]
                self.save_models()
            
            # Удаляем виджеты
            if model_name in self.model_widgets:
                del self.model_widgets[model_name]
            
            # Удаляем рамку
            frame.destroy()
    
    def load_models(self):
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    data = json.load(f)
                    
                    # Конвертируем старый формат (список) в новый (словарь)
                    if isinstance(data, list):
                        # Старый формат - просто список имен моделей
                        new_data = {}
                        for model_name in data:
                            new_data[model_name] = {
                                'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
                            }
                        return new_data
                    else:
                        # Новый формат - словарь
                        return data
            except Exception as e:
                print(f"Ошибка загрузки: {e}")
                return {}
        return {}
    
    def save_models(self):
        try:
            with open(self.config_file, 'w') as f:
                json.dump(self.models_data, f, indent=2)
        except Exception as e:
            print(f"Ошибка сохранения: {e}")

# Запуск программы
if __name__ == "__main__":
    root = tk.Tk()
    app = GazeboController(root)
    root.mainloop()