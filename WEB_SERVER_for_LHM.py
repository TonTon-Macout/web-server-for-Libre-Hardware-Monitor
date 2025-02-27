VERSION = "0.08 beta"
NAME = "WEB_Server_for_LHM" 
import win32com.client
from pythoncom import CoInitialize, CoUninitialize
import sys
import time
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
    ip_list = get_all_local_ips()
    host = get_most_likely_ip(ip_list)
    default_settings = {
        "lhm_url": f"http://{host}:8085/data.json",
        "server_url": f"http://{host}:5001/filtered_data",
        "autostart": False,
        "debug": False  # По умолчанию отладка выключена
    }
    
    try:
        with open(settings_file, "r", encoding="utf-8") as file:
            settings = json.load(file)
            if settings:
                # Убеждаемся, что "debug" есть в настройках
                if "debug" not in settings:
                    settings["debug"] = False
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
        response = requests.get(url, timeout=5)
        response.raise_for_status()  # Проверка на ошибки HTTP
        data = response.json()
        error = ""  # Очищаем ошибку, если всё успешно
        return data
    except requests.exceptions.RequestException as e:
        error = f"LHM сервер недоступен: {str(e)}"
        return None  # Возвращаем None, но с описанием ошибки в глобальной переменной error


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
    paths_dict = ui.get_filtered_data()
    
    if data is None:
        # Если LHM недоступен, возвращаем JSON с ошибкой
        return jsonify({"ERROR": error or "LHM сервер недоступен, данные отсутствуют"}), 503  # 503 Service Unavailable
    else:
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

import psutil  # Добавьте этот импорт в начало файла, если его там нет

def get_all_local_ips():
    """Возвращает список всех локальных IP-адресов с помощью psutil."""
    ip_list = []
    try:
        # Получаем информацию о сетевых интерфейсах
        interfaces = psutil.net_if_addrs()
        for interface_name, addresses in interfaces.items():
            for address in addresses:
                # Проверяем, что это IPv4 и не link-local (169.254.x.x)
                if address.family == socket.AF_INET and not address.address.startswith("169.254"):
                    if address.address != "127.0.0.1":  # Исключаем loopback, если есть другие варианты
                        ip_list.append(address.address)
                    elif not ip_list:  # Добавляем 127.0.0.1 только если других нет
                        ip_list.append(address.address)
    except Exception as e:
        print(f"Ошибка при получении IP-адресов: {e}")
        ip_list.append("127.0.0.1")  # Запасной вариант
    return ip_list

def get_most_likely_ip(ip_list):
    """Возвращает наиболее вероятный IP-адрес (первый не-loopback, иначе 127.0.0.1)."""
    for ip in ip_list:
        if ip != "127.0.0.1":
            return ip
    return ip_list[0] if ip_list else "127.0.0.1"



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
        # Проверяем, существует ли файл paths.json
        if not os.path.exists(paths_file):
            QMessageBox.warning(ui, "Warning", "Файл paths.json не найден. Пожалуйста, добавьте и сохраните параметры перед созданием скетча.")
            return
        
        with open(paths_file, "r", encoding="utf-8") as file:
            paths = json.load(file)
        
        if not paths:
            QMessageBox.warning(ui, "Warning", "В файле paths.json нет параметров")
            return
        
        # Получаем WiFi данные
        wifi_ssid = ui.ssid_line.text()
        wifi_pass = ui.pass_line.text()
        
        # Вычисляем IP-адрес для serverUrl
        server_ip = get_most_likely_ip(get_all_local_ips())
        
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
            
            variables_declaration += f"{data_type} pc_{param_name};\n"
            variables_assignment += f"""        // Получаем {param_name}
        String {param_name}_str = doc["{param_name}"];
        pc_{param_name} = {param_name}_str{conversion};
        Serial.print("{param_name}: ");
        Serial.println(pc_{param_name});\n"""

        # Формируем полный скетч, используя только f-строки
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
const char* serverUrl = "http://{server_ip}:5001/filtered_data";  // URL сервера с данными
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
        # Подставляем значения напрямую в f-строку
        sketch_code = sketch_code.replace("{wifi_ssid}", wifi_ssid).replace("{wifi_pass}", wifi_pass).replace("{server_ip}", server_ip)

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
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("О программе")
        self.setFixedSize(400, 250)  # Размер окна

        # Layout для окна
        layout = QVBoxLayout()

        about_text = QTextBrowser()
        about_text.setOpenExternalLinks(True)  # Позволяет открывать ссылки в браузере
        about_text.setText(
            f"""
            <h2>Программа: {NAME}</h2>
            <p><b>Версия:</b> {VERSION}</p>
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


def resizeEvent(self, event):
    super().resizeEvent(event) 

    # Получаем текущий размер окна
    new_size = event.size()
    new_width = new_size.width()
    new_height = new_size.height()

    # Базовые размеры окна (изначальные)
    base_width = 500
    base_height = 750

    # Рассчитываем масштаб на основе ширины или высоты
    width_scale = new_width / base_width
    height_scale = new_height / base_height

    # Используем минимальный масштаб, чтобы сохранить пропорции
    new_scale = min(width_scale, height_scale)

    # Применяем новый масштаб к браузеру
    self.browser.setZoomFactor(new_scale)    

class MainWindow(QWidget):
    def __init__(self):
        # Проверяем, запущена ли программа через автозагрузку
        is_autostart = "--autostart" in sys.argv
        if is_autostart:
            print("Запуск через автозагрузку, добавляем задержку 10 секунд")
            time.sleep(10)

        super().__init__()
        self.setWindowTitle(NAME)
        self.server_running = False
        self.server_thread = None
        self.shutdown_thread = None
        self.parameter_groups = []
        self.is_closing = False
        
        self.shared_memory = QSharedMemory("WebServerForLHM")
        if not self.shared_memory.create(1):
            if not is_autostart:
                QMessageBox.critical(None, "Ошибка", "Приложение уже запущено!")
            else:
                print("Приложение уже запущено, завершаем автозапуск")
                with open("debug.log", "a", encoding='utf-8') as f:
                    f.write(f"Приложение уже запущено, автозапуск отменён: {time.ctime()}\n")
            sys.exit(1)

        self.setMinimumSize(500, 600)
        self.initUI()
        self.create_tray_icon()

        # Запускаем сервер при автозапуске
        if is_autostart and not self.server_running:
            print("Автозапуск сервера")
            server_thread = Thread(target=self.start_server, daemon=True)
            server_thread.start()
            # Даём время серверу стартовать перед продолжением
            time.sleep(2)


    def initUI(self):
        self.main_layout = QVBoxLayout()

        # Загружаем настройки
        settings = load_settings()
         # Чекбокс "при старте"
        self.autostart_cb = QCheckBox("автозапуск")
        self.autostart_cb.stateChanged.connect(self.toggle_autostart)
        # Проверяем наличие задачи при запуске и синхронизируем чекбокс
        real_autostart = self.check_autostart_status()
        self.autostart_cb.setChecked(real_autostart)
        if settings.get("autostart") != real_autostart:
            self.save_autostart_setting(real_autostart)

# первая строчка

       
    # Чекбокс "Поверх всех окон"
        self.always_on_top_cb = QCheckBox("поверх")  
        self.always_on_top_cb.stateChanged.connect(self.toggle_always_on_top)
        
        # Разделитель
        spacer = QWidget()
        spacer.setFixedSize(28, 0)

       # URL поле LHM как выпадающий список
        url_label = QLabel("LHM URL:")
        self.url_line = QComboBox()  # Используем QComboBox вместо QLineEdit
        self.url_line.setEditable(True)  # Разрешаем редактирование вручную
        #self.url_line.setMinimumWidth(200)  # Устанавливаем минимальную ширину для видимости
        ip_list = get_all_local_ips()  # Получаем все IP-адреса
        most_likely_ip = get_most_likely_ip(ip_list)
        # Добавляем варианты в выпадающий список
        self.url_line.addItems([f"http://{ip}:8085/data.json" for ip in ip_list])
        # Устанавливаем начальное значение
        self.url_line.setCurrentText(f"http://{most_likely_ip}:8085/data.json")

    # Диагностика: выводим список IP-адресов в консоль
        print("Доступные IP-адреса:", ip_list)
        
    # Кнопка "Открыть LHM сервер"
        self.open_lhm_server_btn = QPushButton("Открыть")
        self.open_lhm_server_btn.clicked.connect(self.open_lhm_server)
        
        # Добавляем виджеты в layout
        url_layout = QHBoxLayout()# layout для URL и чекбокса поверх
        ###
        url_layout = QHBoxLayout()
        url_layout.addWidget(self.always_on_top_cb)
        url_layout.addWidget(spacer)
        url_layout.addWidget(url_label)
        url_layout.addWidget(self.url_line, stretch=1) 
        url_layout.addWidget(self.open_lhm_server_btn)
       
        self.main_layout.addLayout(url_layout) # Добавляем в основной

# вторая строчка
    # Чекбокс "при старте"

        # Загружаем состояние из настроек
        self.autostart_cb.setChecked(settings.get("autostart", False))                

        
    # label
        url_server_label = QLabel("Server URL:")
        self.url_server_line = QLineEdit()
        url_server = f"http://{most_likely_ip}:5001/filtered_data"  # Используем тот же IP для сервера
        self.url_server_line.setText(settings.get("server_url", url_server))
    # Открыть    
        self.open_browser_btn = QPushButton("Открыть")
        self.open_browser_btn.clicked.connect(self.open_browser)
        self.open_browser_btn.setEnabled(False)
        self.url_server_line.setEnabled(True)
        

        # Добавляем виджеты в layout
        url_server_layout = QHBoxLayout() # 
        ###
        url_server_layout.addWidget(self.autostart_cb)
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
    # ссылка
        label_parse = QLabel()
        label_parse.setText('<a href="https://jsonformatter.org/json-parser">jsonformatter.org</a>')
        label_parse.setOpenExternalLinks(True)  # Открывать ссылки в браузере
    # label
        ssid_label = QLabel("*WiFi:")
    # ssid
        self.ssid_line = QLineEdit()
        self.ssid_line.setPlaceholderText("Введите имя WiFi сети")
    # Пароль
        pass_label = QLabel("")
        self.pass_line = QLineEdit()
        self.pass_line.setPlaceholderText("Введите пароль WiFi")
        self.pass_line.setEchoMode(QLineEdit.EchoMode.Password)
        
        # Добавляем виджеты в layout
        wifi_layout = QHBoxLayout()       
        ###
        wifi_layout.addWidget(label_parse)
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


    def check_server_status(self):
        """Проверяет, работает ли сервер, пытаясь сделать запрос к filtered_data"""
        server_url = self.url_server_line.text() if hasattr(self, 'url_server_line') else "http://localhost:5001/filtered_data"
        try:
            response = requests.get(server_url, timeout=2)
            if response.status_code == 200:
                print(f"Сервер доступен на {server_url}")
                return True
            else:
                print(f"Сервер вернул код {response.status_code}")
                return False
        except requests.RequestException as e:
            print(f"Сервер не отвечает: {e}")
            return False


    def show(self):
        if self.server_running:
            self.start_stop_btn.setText("Остановить")
            self.start_stop_btn.setEnabled(True)
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
        else:
            self.start_stop_btn.setText("Запустить")
            self.start_stop_btn.setEnabled(True)
            self.start_stop_btn.setStyleSheet("")
            self.open_browser_btn.setEnabled(False)
            self.open_browser_btn.setStyleSheet("")
        
        super().show()



    def check_autostart_status(self):
        import win32com.client
        from pythoncom import CoInitialize, CoUninitialize

        task_name = "WEB_Server_for_LHM_Autostart"
        CoInitialize()
        try:
            scheduler = win32com.client.Dispatch("Schedule.Service")
            scheduler.Connect()
            folder = scheduler.GetFolder("\\")
            folder.GetTask(task_name)
            CoUninitialize()
            return True
        except Exception:
            CoUninitialize()
            return False


    def toggle_autostart(self, state):
        import os
        import win32com.client
        from pythoncom import CoInitialize, CoUninitialize

        app_path = os.path.abspath(sys.argv[0])  # Путь к .exe
        app_dir = os.path.dirname(app_path)  # Папка с .exe
        print(f"Проверяемый путь: {app_path}")
        if not os.path.exists(app_path):
            QMessageBox.warning(self, "Warning", f"Файл не найден: {app_path}")
            self.autostart_cb.setChecked(False)
            self.save_autostart_setting(False)
            return

        task_name = "WEB_Server_for_LHM_Autostart"
        current_user = os.getlogin()
        task_exists = self.check_autostart_status()

        CoInitialize()
        try:
            scheduler = win32com.client.Dispatch("Schedule.Service")
            scheduler.Connect()

            if state == Qt.CheckState.Checked.value:
                if not task_exists:
                    try:
                        print("Создание новой задачи...")
                        task_def = scheduler.NewTask(0)

                        print("Настройка триггера...")
                        trigger = task_def.Triggers.Create(9)  # TASK_TRIGGER_LOGON
                        trigger.UserId = current_user
                        print(f"Триггер настроен для пользователя: {current_user}")

                        print("Настройка действия...")
                        action = task_def.Actions.Create(0)  # TASK_ACTION_EXEC
                        action.Path = app_path
                        action.Arguments = "--autostart"
                        action.WorkingDirectory = app_dir  # Указываем рабочую директорию
                        print(f"Действие настроено: {app_path} --autostart")
                        print(f"Рабочая директория: {app_dir}")

                        print("Настройка параметров задачи...")
                        task_def.Settings.Enabled = True
                        print("Enabled = True")

                        print("Регистрация задачи...")
                        folder = scheduler.GetFolder("\\")
                        folder.RegisterTaskDefinition(
                            task_name, task_def, 6, None, None, 0  # TASK_CREATE_OR_UPDATE, TASK_LOGON_NONE
                        )
                        print("Задача успешно зарегистрирована")
                        with open("debug.log", "a", encoding='utf-8') as f:
                            f.write(f"Задача добавлена: {time.ctime()}\n")
                        QMessageBox.information(self, "Info", "Задача автозапуска создана.")
                    except Exception as e:
                        error_msg = f"Ошибка создания задачи: {str(e)}"
                        print(error_msg)
                        QMessageBox.warning(self, "Warning", error_msg)
                        self.autostart_cb.setChecked(False)
                        self.save_autostart_setting(False)
                        return
                else:
                    print("Задача уже существует, ничего не делаем")
            else:
                if task_exists:
                    try:
                        print("Удаление задачи...")
                        folder = scheduler.GetFolder("\\")
                        folder.DeleteTask(task_name, 0)
                        print("Задача успешно удалена")
                        with open("debug.log", "a", encoding='utf-8') as f:
                            f.write(f"Задача удалена: {time.ctime()}\n")
                        QMessageBox.information(self, "Info", "Задача автозапуска удалена.")
                    except Exception as e:
                        error_msg = f"Ошибка удаления задачи: {str(e)}"
                        print(error_msg)
                        QMessageBox.warning(self, "Warning", error_msg)
                        self.autostart_cb.setChecked(True)
                        self.save_autostart_setting(True)
                        return
                else:
                    print("Задачи нет, ничего не делаем")
        except Exception as e:
            error_msg = f"Ошибка подключения к Task Scheduler: {str(e)}"
            print(error_msg)
            QMessageBox.warning(self, "Warning", error_msg)
            return
        finally:
            CoUninitialize()

        self.save_autostart_setting(state == Qt.CheckState.Checked.value)



    def save_autostart_setting(self, autostart_state):
        settings = load_settings()  # Загружаем текущие настройки
        settings["autostart"] = autostart_state  # Обновляем только autostart
        with open("settings.json", "w", encoding="utf-8") as file:
            json.dump(settings, file, ensure_ascii=False, indent=4)
        print(f"Состояние автозагрузки сохранено: {autostart_state}")
  
    def save_settings(self):
        lhm_url = self.url_line.currentText()
        server_url = self.url_server_line.text()
        autostart = self.autostart_cb.isChecked()  # Учитываем текущее состояние чекбокса
        if lhm_url and server_url:
            settings = {
                "lhm_url": lhm_url,
                "server_url": server_url,
                "autostart": autostart
            }
            with open("settings.json", "w", encoding="utf-8") as file:
                json.dump(settings, file, ensure_ascii=False, indent=4)
            QMessageBox.information(self, "Info", "Настройки сохранены успешно.")
        else:
            QMessageBox.warning(self, "Warning", "Пожалуйста, заполните оба поля.")
    
    def show_about_dialog(self):
            about_dialog = AboutDialog()
            about_dialog.exec()


    def create_tray_icon(self):
        self.tray_icon = QSystemTrayIcon(self)
        self.tray_icon.setIcon(QIcon("icon.png"))  
    
        self.tray_menu = QMenu(self)
    
        restore_action = QAction("Развернуть", self)
        restore_action.triggered.connect(self.show)
        self.tray_menu.addAction(restore_action)
    
        self.server_action = QAction("Запустить сервер", self)
        self.server_action.triggered.connect(self.toggle_server_from_tray)
        self.tray_menu.addAction(self.server_action)
    
        quit_action = QAction("Выход", self)
        quit_action.triggered.connect(self.quit_application)  # Уже вызывает правильный метод
        self.tray_menu.addAction(quit_action)
    
        self.tray_icon.setContextMenu(self.tray_menu)
        self.tray_icon.activated.connect(self.on_tray_icon_activated)
        self.tray_icon.show()
        self.update_tray_menu()


    def update_tray_menu(self):
        if self.server_running:
            self.server_action.setText("Остановить сервер")
        else:
            self.server_action.setText("Запустить сервер")
   
    def toggle_server_from_tray(self):
        if self.server_running:
            self.stop_server()
        else:
            self.start_server()   

    def stop_server_from_tray(self):
        if self.server_running:
            self.stop_server()

    def on_tray_icon_activated(self, reason):
        if reason == QSystemTrayIcon.ActivationReason.Trigger:
            self.show()

    def closeEvent(self, event):
        if not self.is_closing:
            if self.server_running:
                # Если сервер запущен, сворачиваем в трей
                event.ignore()
                self.hide()
                tooltip = ToolTip(
                    "Сервер запущен",
                    "Приложение свернуто в трей<br>Cервер продолжает работать.",
                    self
                )
                QTimer.singleShot(100, lambda: tooltip.show_near_tray(self.tray_icon))
            else:
                # Если сервер не запущен, закрываем приложение полностью
                self.is_closing = True
                event.accept()  # Разрешаем закрытие
                QApplication.instance().quit()  # Завершаем приложение
        else:
            event.accept()  # Если is_closing уже True, закрываем
   
    def open_lhm_server(self):
        webbrowser.open(self.url_line.currentText())  # Заменяем text() на currentText()

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
        lhm_url = self.url_line.currentText()
        server_url = self.url_server_line.text()
        is_autostart = "--autostart" in sys.argv
        settings = load_settings()
        debug_enabled = settings.get("debug", False)  # Читаем настройку "debug"
        
        if debug_enabled:
            print("Начало запуска сервера...")
            with open("debug.log", "a", encoding='utf-8') as f:
                f.write(f"Начало запуска сервера: {time.ctime()}\n")

        if lhm_url and server_url:
            if debug_enabled:
                print(f"Проверка доступности LHM на {lhm_url}...")
            try:
                response = requests.get(lhm_url, timeout=5)
                if response.status_code != 200:
                    raise Exception(f"Код {response.status_code}")
                if debug_enabled:
                    print("LHM сервер доступен")
                    with open("debug.log", "a", encoding='utf-8') as f:
                        f.write(f"LHM сервер доступен: {time.ctime()}\n")
            except requests.RequestException as e:
                error_msg = f"Сервер LHM не запущен по адресу {lhm_url}. Проверьте, запущен ли он, и попробуйте снова."
                if debug_enabled:
                    print(error_msg)
                    with open("debug.log", "a", encoding='utf-8') as f:
                        f.write(f"Ошибка: {str(e)} - {time.ctime()}\n")
                if not is_autostart:
                    msg_box = QMessageBox(self)
                    msg_box.setWindowTitle("Ошибка")
                    msg_box.setText(error_msg)
                    msg_box.setStandardButtons(QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Ignore)
                    ok_button = msg_box.button(QMessageBox.StandardButton.Ok)
                    ok_button.setText("ОК")
                    ignore_button = msg_box.button(QMessageBox.StandardButton.Ignore)
                    ignore_button.setText("Запустить всё равно")
                    msg_box.setDefaultButton(QMessageBox.StandardButton.Ok)
                    result = msg_box.exec()
                    if result == QMessageBox.StandardButton.Ok:
                        return
                    if debug_enabled:
                        print("Пользователь выбрал запуск несмотря на ошибку LHM")
                        with open("debug.log", "a", encoding='utf-8') as f:
                            f.write(f"Запуск сервера несмотря на недоступность LHM: {time.ctime()}\n")
                else:
                    if debug_enabled:
                        print("Прерываем запуск сервера из-за недоступности LHM")
                    return

            try:
                host, port = extract_host_and_port(server_url)
                if host in ["localhost", "127.0.0.1", ""]:
                    host = get_most_likely_ip(get_all_local_ips())
                self.url_server_line.setText(f"http://{host}:{port}/filtered_data")
                
                self.server_thread = FlaskServerThread(host, port)
                self.server_thread.daemon = True
                self.server_thread.start()
                self.server_running = True
                
                if debug_enabled:
                    print("Серверный поток запущен")
                    with open("debug.log", "a", encoding='utf-8') as f:
                        f.write(f"Серверный поток запущен: {time.ctime()}\n")
                
                self.start_stop_btn.setText("Остановить")
                self.start_stop_btn.setEnabled(True)
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
                self.update_tray_menu()
                
                if not is_autostart:
                    QMessageBox.information(self, "Info", f"Сервер запущен на {host}:{port}!")
                else:
                    if debug_enabled:
                        print(f"Сервер запущен на {host}:{port} (автозапуск)")
                        with open("debug.log", "a", encoding='utf-8') as f:
                            f.write(f"Сервер запущен (автозапуск): {time.ctime()}\n")
            except Exception as e:
                error_msg = f"Ошибка при запуске сервера: {str(e)}"
                if debug_enabled:
                    print(error_msg)
                    with open("debug.log", "a", encoding='utf-8') as f:
                        f.write(f"{error_msg} - {time.ctime()}\n")
                if not is_autostart:
                    QMessageBox.critical(self, "Error", error_msg)
        else:
            error_msg = "Ошибка: некорректные URL при запуске сервера"
            if debug_enabled:
                print(error_msg)
                with open("debug.log", "a", encoding='utf-8') as f:
                    f.write(f"{error_msg} - {time.ctime()}\n")
            if not is_autostart:
                QMessageBox.critical(self, "Error", "Пожалуйста, введите корректные URL.")


    def on_server_stopped(self):
        self.server_running = False
        self.start_stop_btn.setEnabled(True)
        self.start_stop_btn.setText("Запустить")
        self.start_stop_btn.setStyleSheet("")
        self.open_browser_btn.setEnabled(False)
        self.open_browser_btn.setStyleSheet("")
        self.server_thread = None
        self.shutdown_thread = None
        self.update_tray_menu()
        if "--autostart" not in sys.argv:
            QMessageBox.information(self, "Info", "Сервер остановлен.")
        else:
            print("Сервер остановлен (автозапуск)")
            with open("debug.log", "a", encoding='utf-8') as f:
                f.write(f"Сервер остановлен (автозапуск): {time.ctime()}\n")
    
    
    def quit_application(self):
        self.is_closing = True  # Устанавливаем флаг закрытия
        if self.server_running:
            self.stop_server()  # Останавливаем сервер, если он запущен
            if self.shutdown_thread:
                self.shutdown_thread.wait(2000)  # Ждём завершения потока остановки
        QApplication.instance().quit()  # Завершаем приложение

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

    def stop_server(self):
        if self.server_thread:
            self.shutdown_thread = ServerShutdownThread(self.server_thread)
            self.shutdown_thread.finished.connect(self.on_server_stopped)
            self.shutdown_thread.error.connect(self.on_stop_error)
            self.shutdown_thread.start()
            self.start_stop_btn.setEnabled(False)



class ToolTip(QWidget):
    def __init__(self, title, message, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.ToolTip | Qt.WindowType.FramelessWindowHint)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        
        layout = QVBoxLayout()
        combined_label = QLabel(f"<b>{title}</b><br> {message}")
        combined_label.setWordWrap(True)
        layout.addWidget(combined_label)
        self.setLayout(layout)
        self.setMinimumWidth(250)  
        self.setStyleSheet("""
            QWidget {
                background-color: #FFFFE0;
                border: 1px solid #DAA520;
                border-radius: 5px;
                padding: 5px;
            }
            QLabel {
                color: black;
            }
        """)

        QTimer.singleShot(3000, self.hide)

    def show_near_tray(self, tray_icon):
        # Получаем геометрию трея
        tray_geometry = tray_icon.geometry()
        offset = 120  # Смещение влево 
        if not tray_geometry.isValid() or tray_geometry.x() == 0 and tray_geometry.y() == 0:
            # Если геометрия недействительна, используем экранный подход
            screen = QApplication.primaryScreen()
            screen_geometry = screen.availableGeometry()
            x = screen_geometry.width() - self.width() - 10 - offset  # Смещаем левее
            y = screen_geometry.height() - self.height() - 40 - 20
        else:
            # Используем координаты трея
            screen = QApplication.screenAt(tray_geometry.center())
            screen_geometry = screen.availableGeometry()
            x = tray_geometry.x() - offset  # Смещаем левее от иконки трея
            y = tray_geometry.y() - self.height()  # Над иконкой
            # Корректируем положение, если выходит за пределы
            if y < screen_geometry.top():
                y = tray_geometry.bottom()  # Под иконкой
            if x + self.width() > screen_geometry.right():
                x = screen_geometry.right() - self.width()
            elif x < screen_geometry.left():
                x = screen_geometry.left()  # Не даём выйти за левый край

        self.move(x, y)
        self.show()






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
    import sys
    import time
    from threading import Thread

    is_autostart = "--autostart" in sys.argv
    app_qt = QApplication(sys.argv)

    ui = MainWindow()

    if is_autostart:
        print("Запуск через автозагрузку, программа остаётся вトレе")
    else:
        ui.show()

    sys.exit(app_qt.exec())