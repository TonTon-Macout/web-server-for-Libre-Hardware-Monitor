import sys
import json
import threading
import requests
import socket
import webbrowser
import os
from flask import Flask, jsonify, request
from werkzeug.serving import make_server

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QMessageBox, QScrollArea, QGroupBox, 
    QComboBox, QFrame, QCheckBox, QDialog, QTextEdit, QTextBrowser,
    QMenu, QSystemTrayIcon
)
from PyQt6.QtCore import QThread, pyqtSignal, QTimer, Qt, QSharedMemory, QBuffer, QIODevice, QDataStream
from PyQt6.QtGui import QIcon, QAction


app = Flask(__name__)
error = ""

# Файл для хранения путей
paths_file = "paths.json"

# Параметр по умолчанию, если файл не найден или пуст
default_paths = {
    "CPU_Temperature": "Children/0/Children/3/Children/3/Children/0/Value"
}



def load_paths():
    try:
        with open(paths_file, "r", encoding="utf-8") as file:
            paths = json.load(file)
            if paths and len(paths) > 0:
                return paths
            return {
                "CPU_Temperature": {
                    "path": "Children/0/Children/3/Children/3/Children/0/Value",
                    "type": "float"
                }
            }
    except (FileNotFoundError, json.JSONDecodeError):
        return {
            "CPU_Temperature": {
                "path": "Children/0/Children/3/Children/3/Children/0/Value",
                "type": "float"
            }
        }
def save_paths(paths_data):
    """
    Сохраняет данные о путях и типах в JSON файл
    paths_data:  {"name": {"path": "path/to/value", "type": "data_type"}}
    """
    with open(paths_file, "w", encoding="utf-8") as file:
        json.dump(paths_data, file, ensure_ascii=False, indent=4)

def load_settings():
    settings_file = "settings.json"
    default_settings = {
        "lhm_url": "http://localhost:8085/data.json",
        "server_url": "http://localhost:5001/filtered_data"
    }
    
    try:
        with open(settings_file, "r", encoding="utf-8") as file:
            settings = json.load(file)
            if settings:
                return settings
            return default_settings
    except (FileNotFoundError, json.JSONDecodeError):
        return default_settings

def save_settings(lhm_url, server_url):
    settings_file = "settings.json"
    settings = {
        "lhm_url": lhm_url,
        "server_url": server_url
    }
    
    with open(settings_file, "w", encoding="utf-8") as file:
        json.dump(settings, file, ensure_ascii=False, indent=4)


def get_data_from_lhm(url):
    global error
    try:
        response = requests.get(url)
        response.raise_for_status()  # Проверка на ошибки HTTP
        data = response.json()
        return data
    except requests.exceptions.RequestException as e:
        error = f"Error when receiving data from LHM: {e}"
        return None

def extract_value(data, path):
    try:
        keys = path.replace('►', '/').split('/')
        for key in keys:
            data = data[int(key)] if key.isdigit() else data[key]
        return data
    except (IndexError, KeyError) as e:
        global error
        error = f"Error when extracting value with path '{path}': {e}"
        return None

def filter_data(data, paths):
    global error
    filtered_data = {key: extract_value(data, path) for key, path in paths.items()}
    filtered_data["ERROR"] = error
    return filtered_data

@app.route('/filtered_data', methods=['GET'])
def get_filtered_data():
    data = get_data_from_lhm(lhm_url)
    # Получаем имя и путь параметра из интерфейса 
    paths_dict = ui.get_filtered_data()
    filtered_data = filter_data(data, paths_dict)

    if filtered_data["ERROR"]:
        with open("error_log.json", "w", encoding="utf-8") as error_file:
            json.dump({"error": filtered_data["ERROR"]}, error_file, ensure_ascii=False, indent=4)
    return jsonify(filtered_data)



class FlaskServerThread(threading.Thread):
    def __init__(self, host, port):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        self.srv = make_server(self.host, self.port, app)
        self.ctx = app.app_context()
        self.ctx.push()

    def run(self):
        self.srv.serve_forever()

    def shutdown(self):
        self.srv.shutdown()
    
def extract_host_and_port(url):
        """
        Извлекает хост и порт из URL.
        Пример:
        "http://192.168.1.100:5001/filtered_data" -> ("192.168.1.100", 5001)
        """
        from urllib.parse import urlparse
        parsed_url = urlparse(url)
        host = parsed_url.hostname
        port = parsed_url.port or 5001  # Если порт не указан, используем 5001 по умолчанию
        return host, port    

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"



def convert_path_to_json_format(path):
    """Преобразует путь в формат JSON с квадратными скобками"""
    # Сначала заменяем стрелки на слеши
    path = path.replace('►', '/')
    # Разбиваем путь на части
    parts = path.split('/')
    # Формируем путь в формате JSON
    return ''.join(f'[{part}]' for part in parts)



def copy_example_sketch():
    try:
        with open(paths_file, "r", encoding="utf-8") as file:
            paths = json.load(file)
        
        if not paths:
            QMessageBox.warning(ui, "Warning", "В файле paths.json нет параметров")
            return
        
        # Получаем WiFi данные
        wifi_ssid = ui.ssid_line.text()
        wifi_pass = ui.pass_line.text()
        
        # Формируем объявления переменных
        variables_declaration = ""
        variables_assignment = ""
        
        for param_name, param_data in paths.items():
            data_type = param_data.get("type", "String")
            conversion = ""
            if data_type == "int":
                conversion = ".toInt()"
            elif data_type == "float":
                conversion = ".toFloat()"
            elif data_type == "bool":
                conversion = ".equals(\"true\")"
            elif data_type in ["long", "double"]:
                conversion = f".to{data_type.capitalize()}()"
            
            # Объявление переменной
            variables_declaration += f"{data_type} pc_{param_name};\n"
            
            # Присвоение значения
            variables_assignment += f"""        // Получаем {param_name}
        String {param_name}_str = doc["{param_name}"];
        pc_{param_name} = {param_name}_str{conversion};
        Serial.print("{param_name}: ");
        Serial.println(pc_{param_name});\n"""

        # Формируем полный скетч
        sketch_code = f"""/*
   Скетч для мониторинга параметров ПК через LibreHardwareMonitor
   для ESP8266 и ESP32
   Автоматически сгенерировано программой WEB Server for LHM
   LHM    - https://github.com/LibreHardwareMonitor/LibreHardwareMonitor
   Сервер - https://github.com/TonTon-Macout/web-server-for-Libre-Hardware-Monitor
*/

#include <Arduino.h>
#include <ArduinoJson.h>      // Библиотека для работы с JSON

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#else
#include <WiFi.h>
#include <HTTPClient.h>
#endif

// Настройки WiFi
const char* ssid = "{wifi_ssid}";  // Имя WiFi сети
const char* password = "{wifi_pass}";     // Пароль от WiFi сети
const char* serverUrl = "http://{get_local_ip()}:5001/filtered_data";  // URL сервера с данными
#define UPDATE_INTERVAL 10000  // Интервал обновления данных (10 секунд)
#define HTTP_TIMEOUT 10000     // 10 секунд
#define MAX_HTTP_RETRIES 5     // Максимальное количество попыток

WiFiClient client;
HTTPClient http;

// Здесь храним переменные 
{variables_declaration}
bool enableRequests = true;  // Флаг разрешения запросов

void setup() {{
  Serial.begin(115200); // Инициализация последовательного порта
  Serial.println("\\n\\nСтартууееем!!!");

  // Проверка настроек WiFi
  if (strlen(ssid) == 0 || strlen(password) == 0) {{
    Serial.println("А данные для WiFi кто вводить будет?");
    Serial.println("Вводи данные в скетче, загружай по новой и возвращайся!");
    Serial.println("подождем");
    while (true) {{
      delay(1000);
      Serial.println(".");
    }}
  }}

  // Подключение к WiFi
  WiFi.begin(ssid, password);
  Serial.print("Подключаемся к WiFi");

  while (WiFi.status() != WL_CONNECTED) {{
    delay(500);
    Serial.print(".");
  }}
  Serial.println("\\nПодключено!!!111одинодин");
  Serial.print("IP адрес: ");
  Serial.println(WiFi.localIP());
}}

void reconnectWiFi() {{
  if (WiFi.status() != WL_CONNECTED) {{
    WiFi.begin(ssid, password);
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {{
      delay(500);
      Serial.print(".");
      attempts++;
    }}
    if (WiFi.status() == WL_CONNECTED) {{
      Serial.println("\\nСнова подключились к WiFi!");
      Serial.print("IP адрес: ");
      Serial.println(WiFi.localIP());
    }} else {{
      Serial.println("\\nПодключение к WiFi не удалось!");
    }}
  }}
}}

bool sendHttpRequest() {{
  http.end(); // Закрываем предыдущее соединение если оно было
  if (!http.begin(client, serverUrl)) {{
    Serial.println("Ошибка инициализации HTTP");
    return false;
  }}
  http.setTimeout(HTTP_TIMEOUT);
  return true;
}}

void PCMonitor() {{
  if (WiFi.status() != WL_CONNECTED) {{
    Serial.println("WiFi не подключен!");
    return;
  }}

  uint8_t retries = 0;// Счетчик попыток
  bool success = false;// Флаг успешности запроса
  // Повторяем запрос пока не получим данные или не достигнем лимита попыток
  while (retries < MAX_HTTP_RETRIES && !success) {{
    if (retries > 0) {{
      Serial.printf("\\nПовторная попытка %d из %d\\n", retries + 1, MAX_HTTP_RETRIES);
      delay(1000); // Пауза между попытками
    }}

    Serial.println("\\nОтправка запроса...");

    if (!sendHttpRequest()) {{
      retries++;
      continue;
    }}

    int httpCode = http.GET();

    if (httpCode > 0) {{
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {{
        success = true;
        JsonDocument doc;// Создаем JSON документ
        DeserializationError Json_error = deserializeJson(doc, http.getStream());

        if (Json_error) {{
          Serial.print("Ошибка парсинга JSON: ");
          Serial.println(Json_error.f_str());
          return;
        }}

        // Проверяем наличие ошибок от сервера
        String Error = doc["ERROR"];
        if (Error.length() > 0) {{
          Serial.print("Ошибка сервера: ");
          Serial.println(Error);
          return;
        }}

        // Получаем и обрабатываем данные
        Serial.println("\\nПолученные значения:");
{variables_assignment}
        doc.clear();  // Очищаем документ
      }} else {{
        Serial.printf("HTTP код ошибки: %d\\n", httpCode);
      }}
    }} else {{
      Serial.print("Ошибка HTTP запроса: ");
      Serial.println(http.errorToString(httpCode).c_str());
    }}

    http.end();
    retries++;
  }}

  if (!success) {{
    Serial.println("\\n------------------------------");
    Serial.println("Не удалось получить данные после всех попыток");
    enableRequests = false;  // Отключаем запросы
    Serial.println("Увы, но запросы остановлены =(\\nДля возобновления попыток отправьте 'start' или 'старт' в монитор порта");
  }}

  Serial.println("\\n------------------------");
}}

bool isWiFiStable() {{
  int32_t rssi = WiFi.RSSI();
  if (rssi >= -50) {{
    return true;  // Отличный сигнал
  }} else if (rssi >= -70) {{
    return true;  // Хороший сигнал
  }} else {{
    Serial.printf("=( Слабенький сигнал WiFi: %d dBm\\n", rssi);
    return false; // Таксе сигнал
  }}
}}

void loop() {{
  // Что-то пришло по Serial
  if (Serial.available()) {{
    String command = Serial.readStringUntil('\\n');
    command.toLowerCase();  // В нижний регистр
    command.trim();         // Убираем пробелы

    if (command == "start" || command == "старт") {{
      enableRequests = true; // Включаем запросы
      Serial.println("Запросы возобновлены");
    }}
  }}

  // Выполняем запросы только если они разрешены
  if (enableRequests) {{
    if (WiFi.status() != WL_CONNECTED || !isWiFiStable()) {{
      reconnectWiFi();
    }}
    PCMonitor();
    delay(UPDATE_INTERVAL);
  }}
}}
"""
        clipboard = QApplication.clipboard()
        clipboard.setText(sketch_code)
        QMessageBox.information(ui, "Info", "Скетч скопирован в буфер обмена.")
        
    except Exception as e:
        QMessageBox.warning(ui, "Warning", f"Ошибка при создании скетча: {e}")



def open_lhm_server(self):
    webbrowser.open(self.url_line.text())

def toggle_always_on_top(self, state):
    if state == Qt.CheckState.Checked.value:
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.WindowStaysOnTopHint)
    else:
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowType.WindowStaysOnTopHint)
    self.show()  

# Глобальная переменная для URL LHM
lhm_url = ""

class ServerShutdownThread(QThread):
    finished = pyqtSignal()
    error = pyqtSignal(str)
    
    def __init__(self, server_thread):
        super().__init__()
        self.server_thread = server_thread
        
    def run(self):
        try:
            self.server_thread.shutdown()
            self.server_thread.join(timeout=1)
            self.finished.emit()
        except Exception as e:
            self.error.emit(str(e))

def normalize_path(path):
    """Преобразует путь, заменяя стрелки на слеши"""
    return path.replace('►', '/').strip()

class ParameterGroup(QWidget):
    deleted = pyqtSignal(object)  # Добавляем сигнал для уведомления об удалении
    
    def __init__(self, param_name="", param_path="", parent=None):
        super().__init__(parent)
        self.initUI(param_name, param_path)

    def validate_input(self):
        """Проверка корректности введенных данных"""
        param_name = self.param_name_line.text()
        param_path = self.param_path_line.text()

        errors = []

        # Проверка имени параметра
        if param_name:
            if param_name[0].isdigit():
                errors.append("Имя параметра не должно начинаться с цифры")
            if not param_name.isascii():
                errors.append("Имя параметра должно содержать только латинские символы")
            if ' ' in param_name:
                errors.append("Имя параметра не должно содержать пробелы")

        # Проверка пути
        if param_path:
            # Заменяем стрелки и слеши 
            path_without_separators = param_path.replace('►', '').replace('/', '')
            if not path_without_separators.isascii():
                errors.append("Путь должен содержать только латинские символы, стрелки (►) или слеши (/)")
            if ' ' in param_path:
                errors.append("Путь не должен содержать пробелы")

            # Проверяем части пути
            parts = param_path.replace('►', '/').split('/')
            for part in parts:
                if part and not part.replace('_', '').isalnum():
                    errors.append("Части пути должны содержать только буквы, цифры или нижнее подчеркивание")
                    break
                
        return errors


    def on_text_changed(self):
        """Обработчик изменения текста в полях ввода"""
        errors = self.validate_input()
        if errors:
            # Показываем ошибки красным цветом под полями ввода
            self.error_label.setText('\n'.join(errors))
            self.error_label.setStyleSheet("color: red;")
        else:
            self.error_label.clear()

    def initUI(self, param_name, param_path):
        layout = QVBoxLayout()
        
        # Группа параметра
        param_group = QGroupBox()
        group_layout = QVBoxLayout()

        # Имя параметра
        param_name_layout = QHBoxLayout()
        param_name_label = QLabel("Имя параметра:")
        self.param_name_line = QLineEdit()
        self.param_name_line.setText(param_name)
        param_name_layout.addWidget(param_name_label)
        param_name_layout.addWidget(self.param_name_line)
        group_layout.addLayout(param_name_layout)

        # Путь параметра
        param_path_layout = QHBoxLayout()
        param_path_label = QLabel("Путь в LHM:")
        self.param_path_line = QLineEdit()
        self.param_path_line.setText(param_path)
        param_path_layout.addWidget(param_path_label)
        param_path_layout.addWidget(self.param_path_line)
        group_layout.addLayout(param_path_layout)

                # Добавляем горизонтальный разделитель
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        group_layout.addWidget(line)

        # JSON формат
        json_layout = QHBoxLayout()
        json_label = QLabel("Путь в формате JSON:")
        self.json_format_line = QLineEdit()
        self.json_format_line.setReadOnly(True)
        json_layout.addWidget(json_label)
        json_layout.addWidget(self.json_format_line)
        group_layout.addLayout(json_layout)

        # Добавляем горизонтальный layout для типа данных и кнопки копирования
        data_type_layout = QHBoxLayout()
        
        # Тип данных
        data_type_label = QLabel("*Тип данных:")
        self.data_type_combo = QComboBox()
        self.data_type_combo.addItems(["int", "float", "String", "bool", "long", "double"])
        data_type_layout.addWidget(data_type_label)
        data_type_layout.addWidget(self.data_type_combo)
        
        
        # Добавляем разделитель 
        data_type_layout.addStretch()
        
        # Кнопка копирования
        copy_btn = QPushButton("Копировать путь")
        copy_btn.clicked.connect(self.copy_json_format)
        data_type_layout.addWidget(copy_btn)
        
        group_layout.addLayout(data_type_layout)
                        # Добавляем горизонтальный разделитель
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        group_layout.addWidget(line)

        # Добавляем горизонтальный layout для кнопок
        buttons_layout = QHBoxLayout()
        
        # Кнопка сохранения параметра
        save_btn = QPushButton("Сохранить")
        save_btn.clicked.connect(self.save_parameter)
        buttons_layout.addWidget(save_btn)

        # Кнопка удаления параметра
        delete_btn = QPushButton("Удалить")
        delete_btn.clicked.connect(self.delete_parameter)
        buttons_layout.addWidget(delete_btn)
        
        group_layout.addLayout(buttons_layout)

        param_group.setLayout(group_layout)
        layout.addWidget(param_group)
        self.setLayout(layout)

        # Обновление JSON при изменении пути
        self.param_path_line.textChanged.connect(self.update_json_format)
        self.update_json_format()

        # Добавляем поле для вывода ошибок
        self.error_label = QLabel()
        self.error_label.setWordWrap(True)
        group_layout.addWidget(self.error_label)

        # Подключаем обработчики изменения текста
        self.param_name_line.textChanged.connect(self.on_text_changed)
        self.param_path_line.textChanged.connect(self.on_text_changed)

    def update_json_format(self):
        path = self.param_path_line.text()
        json_path = convert_path_to_json_format(path)
        self.json_format_line.setText(json_path)

    def copy_json_format(self):
        clipboard = QApplication.clipboard()
        clipboard.setText(self.json_format_line.text())
        QMessageBox.information(self, "Info", "Путь в JSON формате скопирован в буфер обмена.")


    def delete_parameter(self):
        try:
            # Удаляем параметр из файла
            with open(paths_file, "r", encoding="utf-8") as file:
                paths = json.load(file)
            # Получаем имя параметра для удаления
            param_name = self.param_name_line.text()
            if param_name in paths:
                del paths[param_name]
                # Сохраняем обновленный файл
                with open(paths_file, "w", encoding="utf-8") as file:
                    json.dump(paths, file, ensure_ascii=False, indent=4)
            # Отправляем сигнал и удаляем виджет
            self.deleted.emit(self)
            self.setParent(None)
            self.deleteLater()
        except Exception as e:
            QMessageBox.warning(self, "Warning", f"Ошибка при удалении параметра: {e}")


    def save_parameter(self):
        data = self.get_parameter_data()
        if data["name"] and data["path"]:
            try:
                with open(paths_file, "r", encoding="utf-8") as file:
                    paths = json.load(file)
            except (FileNotFoundError, json.JSONDecodeError):
                paths = {}

            # Сохраняем параметр с типом данных
            paths[data["name"]] = {
                "path": data["path"],
                "type": data["type"]
            }

            with open(paths_file, "w", encoding="utf-8") as file:
                json.dump(paths, file, ensure_ascii=False, indent=4)

            QMessageBox.information(self, "Info", "Параметр сохранен успешно.")
        else:
            QMessageBox.warning(self, "Warning", "Заполните имя и путь параметра.")




    def get_parameter_data(self):
        return {
            "name": self.param_name_line.text(),
            "path": normalize_path(self.param_path_line.text()),
            "type": self.data_type_combo.currentText()
        }

class AboutDialog(QDialog):
    """Окно 'О программе'."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("О программе")
        self.setFixedSize(400, 250)  # Увеличим размер окна для удобства

        # Layout для окна
        layout = QVBoxLayout()

        # Текстовая информация с HTML-форматированием
        about_text = QTextBrowser()
        about_text.setOpenExternalLinks(True)  # Позволяет открывать ссылки в браузере
        about_text.setText(
            """
            <h2>Программа: Web Server for LHM</h2>
            <p><b>Версия:</b> 0.3 beta</p>
            <p><b>Автор:</b> Vanila</p>
            <p><b>Описание:</b> Промежуточный сервер для Libre Hardware Monitor. 
            <ul>
             <li>Создает сервер с JSON данными для передачи в ESP<br>
                 на основе данных из сервера LHM, но в более компактном виде.
             </li>
             <li>Создает скетч на основе введенных данных для ESP8266 и ESP32 в ардуино среде</li>
             <li>Создает переменные для вставки в код на основе введенных данных</li>
             <li>А также упрощает создание Json пути совместно с сайтом<br> <a href="https://jsonformatter.org/json-parser">jsonformatter.org</a></li>
            </ul>
            <p><b>Ссылка на проект:</b> <a href="https://github.com/TonTon-Macout/web-server-for-Libre-Hardware-Monitor">GitHub</a></p>
            <p><br>* Поля помеченные нужны только для формирования скетча,</b> на работу сервера они не влияют</p>
            """
        )
        layout.addWidget(about_text)

        # Кнопка "Закрыть"
        close_button = QPushButton("Закрыть")
        close_button.clicked.connect(self.close)
        layout.addWidget(close_button)

        self.setLayout(layout)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WEB Server for LHM")
        self.server_running = False
        self.server_thread = None
        self.shutdown_thread = None
        self.parameter_groups = []
        self.shared_memory = QSharedMemory("WebServerForLHM")
        if not self.shared_memory.create(1):
            QMessageBox.critical(None, "Ошибка", "Приложение уже запущено!")
            self.shared_memory.attach()
            buffer = QBuffer()
            buffer.open(QIODevice.OpenModeFlag.ReadWrite)
            self.shared_memory.lock()
            buffer.setData(self.shared_memory.data())
            buffer.seek(0)
            stream = QDataStream(buffer)
            stream.readBool()
            self.shared_memory.unlock()
            sys.exit(1)
        # Минимальный размер окна
        self.setMinimumSize(600, 800)  # Ширина , Высота
        self.initUI()
        self.create_tray_icon()

    def initUI(self):
        self.main_layout = QVBoxLayout()

        # Загружаем настройки
        settings = load_settings()

# первая строчка

       
    # Чекбокс "Поверх всех окон"
        self.always_on_top_cb = QCheckBox("поверх")  
        self.always_on_top_cb.stateChanged.connect(self.toggle_always_on_top)
        
    # Разделитель
        spacer = QWidget()
        spacer.setFixedSize(28, 0)

     # URL поле LHM
        url_label = QLabel("LHM URL:") # название поля
        self.url_line = QLineEdit()
        local_ip = get_local_ip()
        url = f"http://{local_ip}:8085/data.json"
        self.url_line.setText(url)
        
    # Кнопка "Открыть LHM сервер"
        self.open_lhm_server_btn = QPushButton("Открыть")
        self.open_lhm_server_btn.clicked.connect(self.open_lhm_server)
        
        # Добавляем виджеты в layout
        url_layout = QHBoxLayout()# layout для URL и чекбокса поверх
        ###
        url_layout.addWidget(self.always_on_top_cb)
        url_layout.addWidget(spacer)
        url_layout.addWidget(url_label)
        url_layout.addWidget(self.url_line)        
        url_layout.addWidget(self.open_lhm_server_btn)
       
        self.main_layout.addLayout(url_layout) # Добавляем в основной

# вторая строчка
        
    # ссылка
        label_parse = QLabel()
        label_parse.setText('<a href="https://jsonformatter.org/json-parser">jsonformatter.org</a>')
        label_parse.setOpenExternalLinks(True)  # Открывать ссылки в браузере
        
    # label
        url_server_label = QLabel("Server URL:")
        self.url_server_line = QLineEdit()

        url_server = f"http://{local_ip}:5001/filtered_data"
        #self.url_server_line.setText(url_server)
        self.url_server_line.setText(settings["server_url"])  # Устанавливаем значение из настроек
        
    # Открыть    
        self.open_browser_btn = QPushButton("Открыть")
        self.open_browser_btn.clicked.connect(self.open_browser)
        self.open_browser_btn.setEnabled(False)
        self.url_server_line.setEnabled(True)
        

        # Добавляем виджеты в layout
        url_server_layout = QHBoxLayout() # 
        ###
        url_server_layout.addWidget(label_parse)
        url_server_layout.addWidget(url_server_label)
        url_server_layout.addWidget(self.url_server_line)
        url_server_layout.addWidget(self.open_browser_btn)
        
        self.main_layout.addLayout(url_server_layout)# Добавляем в основной


# третья строчка 
        
    # кнопка 
        self.start_stop_btn = QPushButton("Запустить")
        self.start_stop_btn.clicked.connect(self.toggle_server)
    # кнопка 
        example_btn = QPushButton("Cкетч")
        example_btn.clicked.connect(copy_example_sketch)
    # кнопка 
        vars_btn = QPushButton("Переменные")
        vars_btn.clicked.connect(copy_variables_code)
    # кнопка 
        about_btn = QPushButton("О программе")
        about_btn.clicked.connect(self.show_about_dialog)
        
    # кнопка "Сохранить"
        self.save_settings_btn = QPushButton("Сохранить IP")
        self.save_settings_btn.clicked.connect(self.save_settings)
       
       
        # Добавляем виджеты в layout
        btn_layout = QHBoxLayout()
        ###
        btn_layout.addWidget(self.start_stop_btn)
        btn_layout.addWidget(example_btn)
        btn_layout.addWidget(vars_btn)
        btn_layout.addWidget(about_btn)
        btn_layout.addWidget(self.save_settings_btn)

        self.main_layout.addLayout(btn_layout) # 

# Третья строчка

    # label
        ssid_label = QLabel("*WiFi SSID:")
    # ssid
        self.ssid_line = QLineEdit()
        self.ssid_line.setPlaceholderText("Введите имя WiFi сети")
    # Пароль
        pass_label = QLabel("*Пароль:")
        self.pass_line = QLineEdit()
        self.pass_line.setPlaceholderText("Введите пароль WiFi")
        self.pass_line.setEchoMode(QLineEdit.EchoMode.Password)
        
        # Добавляем виджеты в layout
        wifi_layout = QHBoxLayout()       
        ###
        wifi_layout.addWidget(ssid_label)
        wifi_layout.addWidget(self.ssid_line)       
        wifi_layout.addWidget(pass_label)
        wifi_layout.addWidget(self.pass_line)
        ###
        self.main_layout.addLayout(wifi_layout)

# Разделительная линия
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        self.main_layout.addWidget(line)

# Область с параметрами

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        self.parameters_layout = QVBoxLayout(scroll_widget)
        scroll_widget.setLayout(self.parameters_layout) 
        self.scroll.setWidget(scroll_widget)

        # Загружаем существующие параметры
        paths = load_paths()
        for param_name, param_data in paths.items():
            # Получаем путь и тип из словаря параметра
            param_path = param_data["path"] if isinstance(param_data, dict) else param_data
            param_type = param_data.get("type", "float") if isinstance(param_data, dict) else "float"
            self.add_parameter_group(param_name, param_path, param_type)

        self.main_layout.addWidget(self.scroll)

        # Кнопка добавления параметра
        add_param_btn = QPushButton("Добавить параметр")
        add_param_btn.clicked.connect(self.add_parameter_group)
        self.main_layout.addWidget(add_param_btn)

        self.setLayout(self.main_layout)
    
    def save_settings(self):
        lhm_url = self.url_line.text()
        server_url = self.url_server_line.text()

        if lhm_url and server_url:
            save_settings(lhm_url, server_url)
            QMessageBox.information(self, "Info", "Настройки сохранены успешно.")
        else:
            QMessageBox.warning(self, "Warning", "Пожалуйста, заполните оба поля.")
    
    def show_about_dialog(self):
            about_dialog = AboutDialog()
            about_dialog.exec()

    def create_tray_icon(self):
        self.tray_icon = QSystemTrayIcon(self)
        self.tray_icon.setIcon(QIcon("icon.png"))  

        tray_menu = QMenu(self)
        restore_action = QAction("Развернуть", self)
        restore_action.triggered.connect(self.show)
        tray_menu.addAction(restore_action)

        quit_action = QAction("Выход", self)
        quit_action.triggered.connect(QApplication.instance().quit)
        tray_menu.addAction(quit_action)

        self.tray_icon.setContextMenu(tray_menu)
        self.tray_icon.activated.connect(self.on_tray_icon_activated)
        self.tray_icon.show()

    def on_tray_icon_activated(self, reason):
        if reason == QSystemTrayIcon.ActivationReason.Trigger:
            self.show()

    def closeEvent(self, event):
        event.ignore()
        self.hide()
        self.tray_icon.showMessage(
            "Приложение свернуто",
            "Приложение продолжает работать в трее",
            QSystemTrayIcon.MessageIcon.Information,
            2000
        )

    def open_lhm_server(self):
        webbrowser.open(self.url_line.text())

    def toggle_always_on_top(self, state):
        if state == Qt.CheckState.Checked.value:
            self.setWindowFlags(self.windowFlags() | Qt.WindowType.WindowStaysOnTopHint)
        else:
            self.setWindowFlags(self.windowFlags() & ~Qt.WindowType.WindowStaysOnTopHint)
        self.show()

    def add_parameter_group(self, param_name="", param_path="", param_type="float"):
        if isinstance(param_name, bool):
            param_name = ""
            param_path = ""
            param_type = "float"

        if len(self.parameter_groups) >= 50:
            QMessageBox.warning(self, "Warning", "Достигнут максимум параметров (50)")
            return

        param_group = ParameterGroup(param_name, param_path)
        if param_type:
            param_group.data_type_combo.setCurrentText(param_type)
        param_group.deleted.connect(self.remove_parameter_group)
        self.parameter_groups.append(param_group)
        
        # Добавляем новый параметр
        self.parameters_layout.insertWidget(0, param_group)
        
        # Прокручиваем к новому параметру
        QTimer.singleShot(100, lambda: self.scroll.verticalScrollBar().setValue(0))

    def remove_parameter_group(self, group):
        if group in self.parameter_groups:
            self.parameter_groups.remove(group)

    def open_browser(self):
        server_url = self.url_server_line.text()
        if server_url:
            webbrowser.open(server_url)
        else:
            QMessageBox.warning(self, "Warning", "Пожалуйста, введите Server URL.")

    def save_paths_button(self):
        paths = {}
        groups = self.parameter_groups.copy()

        for group in groups:
            try:
                # Проверяем валидность перед сохранением
                errors = group.validate_input()
                if errors:
                    QMessageBox.warning(self, "Warning", 
                        f"Ошибки в параметре {group.param_name_line.text()}:\n" + 
                        "\n".join(errors))
                    return
                
                data = group.get_parameter_data()
                if data["name"] and data["path"]:
                    paths[data["name"]] = {
                        "path": data["path"],
                        "type": data["type"]
                    }
            except RuntimeError:
                continue
            
        if paths:
            save_paths(paths)
            QMessageBox.information(self, "Info", "Paths saved successfully.")
        else:
            QMessageBox.warning(self, "Warning", "Нет параметров для сохранения")

    def get_filtered_data(self):
        paths_dict = {}
        for group in self.parameter_groups:
            data = group.get_parameter_data()
            if data["name"] and data["path"]:
                paths_dict[data["name"]] = data["path"]
        return paths_dict

    def toggle_server(self):
        if not self.server_running:
            self.start_server()
        else:
            self.stop_server()



    def start_server(self):
        global lhm_url
        lhm_url = self.url_line.text()
        server_url = self.url_server_line.text()

        if lhm_url and server_url:
            try:
                # Извлекаем хост и порт из Server URL
                host, port = extract_host_and_port(server_url)

                # Если хост не указан или это localhost, используем IP компьютера
                if host in ["localhost", "127.0.0.1", ""]:
                    host = get_local_ip()  # Получаем IP компьютера

                # Обновляем Server URL в интерфейсе
                self.url_server_line.setText(f"http://{host}:{port}/filtered_data")

                # Запускаем сервер с указанными хостом и портом
                self.server_thread = FlaskServerThread(host, port)
                self.server_thread.start()
                self.server_running = True
                self.start_stop_btn.setText("Остановить")
                self.start_stop_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #ff4444;
                        color: white;
                        padding: 5px;
                        border: none;
                        border-radius: 3px;
                    }
                    QPushButton:hover {
                        background-color: #ff6666;
                    }
                """)
                self.open_browser_btn.setEnabled(True)
                self.open_browser_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #4CAF50;
                        color: white;
                        padding: 5px;
                        border: none;
                        border-radius: 3px;
                    }
                    QPushButton:hover {
                        background-color: #45a049;
                    }
                """)
                QMessageBox.information(self, "Info", f"Сервер запущен на {host}:{port}!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Ошибка при запуске сервера: {e}")
        else:
            QMessageBox.critical(self, "Error", "Пожалуйста, введите корректные URL.")




    def stop_server(self):
        if self.server_thread:
            self.shutdown_thread = ServerShutdownThread(self.server_thread)
            self.shutdown_thread.finished.connect(self.on_server_stopped)
            self.shutdown_thread.error.connect(self.on_stop_error)
            self.shutdown_thread.start()
            self.start_stop_btn.setEnabled(False)  # Блокируем кнопку 

    def on_server_stopped(self):
        self.server_running = False
        self.start_stop_btn.setEnabled(True)
        self.start_stop_btn.setText("Запустить")
        # Возвращаем стиль кнопки 
        self.start_stop_btn.setStyleSheet("")
        self.open_browser_btn.setEnabled(False)
        #
        self.url_server_line.setEnabled(True)
        # Возвращаем стиль кнопки 
        self.open_browser_btn.setStyleSheet("")
        self.server_thread = None
        self.shutdown_thread = None
        QMessageBox.information(self, "Info", "Сервер остановлен.")

    def on_stop_error(self, error_message):
        self.start_stop_btn.setEnabled(True)
        QMessageBox.warning(self, "Warning", f"Ошибка при остановке сервера: {error_message}")

def copy_variables_code():
    try:
        with open(paths_file, "r", encoding="utf-8") as file:
            paths = json.load(file)
        
        if not paths:
            QMessageBox.warning(ui, "Warning", "В файле paths.json нет параметров")
            return
        
        # Формируем код переменных
        variables_code = ""
        for param_name, param_data in paths.items():
            data_type = param_data.get("type", "String")
            conversion = ""
            if data_type == "int":
                conversion = ".toInt()"
            elif data_type == "float":
                conversion = ".toFloat()"
            elif data_type == "bool":
                conversion = ".equals(\"true\")"
            elif data_type in ["long", "double"]:
                conversion = f".to{data_type.capitalize()}()"
            
            variables_code += f"""   String {param_name} = doc["{param_name}"];
   {data_type} pc_{param_name} = {param_name}{conversion};
"""
        
        clipboard = QApplication.clipboard()
        clipboard.setText(variables_code)
        QMessageBox.information(ui, "Info", "Код переменных скопирован в буфер обмена.")
        
    except Exception as e:
        QMessageBox.warning(ui, "Warning", f"Ошибка при копировании кода переменных: {e}")

if __name__ == '__main__':
    app_qt = QApplication(sys.argv)
    ui = MainWindow()
    ui.show()
    sys.exit(app_qt.exec())