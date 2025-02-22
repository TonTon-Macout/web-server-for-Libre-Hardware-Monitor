@echo off
chcp 65001
cd /d "%~dp0"

echo.
echo ================  Запуск скрипта сборки  ===================
echo ------  Должен быть установлен pithon версии 3 и выше  -----
echo --- а так же библиотеки  jsonlib requests PyQt6 Werkzeug ---

echo.


REM Извлекаем имя и номер версии из скрипта
echo Извлекаем имя и номер версии из модуля Settings...
for /f "delims=" %%i in ('python -c "import WEB_SERVER_for_LHM; print(WEB_SERVER_for_LHM.NAME, WEB_SERVER_for_LHM.VERSION)" 2^>NUL') do set "line=%%i"
if "%line%"=="" (
    echo Ошибка: Не удалось извлечь имя и версию из Settings.py.
    echo Убедитесь, что файл Settings.py существует и содержит переменные NAME и VERSION.
    pause
    exit /b 1
)
echo Извлеченная строка: %line%
for /f "tokens=1,2" %%a in ("%line%") do (
    set "name=%%a"
    set "version=%%b"
)
echo Имя приложения: %name%
echo Версия: %version%
echo.

REM Замена запятых на точки в номере версии (если есть)
set "version=%version:,=.%"
echo.

REM Создаем имя исполняемого файла и папки
set "exe_name=%name%_app_v%version%"
set "dir_name=%exe_name%"
echo Будет создан исполняемый файл: %exe_name%.exe
echo Имя папки для сборки: %dir_name%
echo.

timeout /t 5

REM Запускаем PyInstaller с использованием извлеченного имени и номера версии
echo Запускаем PyInstaller...
start /wait cmd /c python -m PyInstaller --onefile --windowed --icon=icon.ico --name "%exe_name%" WEB_SERVER_for_LHM.py
if %errorlevel% neq 0 (
    echo Ошибка при сборке exe файла.
    pause
    exit /b 1
) else (
    echo PyInstaller завершил работу.
)
echo.

REM Удаление ненужных файлов и папок после сборки
echo Удаляем временные папки и файлы...
rmdir /s /q build
del /q WEB_SERVER_for_LHM.spec
del /q "%exe_name%.spec"

REM Создание папки с именем итогового исполняемого файла
echo Создаем папку %dir_name%...
mkdir "%dir_name%"

REM Перемещаем исполняемый файл в созданную папку
echo Перемещаем %exe_name%.exe в папку %dir_name%...
move "dist\%exe_name%.exe" "%dir_name%"
if %errorlevel% neq 0 (
    echo Не удалось переместить exe файл.
    pause
    exit /b 1
)

REM Копирование дополнительных файлов в созданную папку
echo Копируем дополнительные файлы...
copy "icon.ico" "%dir_name%"
copy "minimize.png" "%dir_name%"
copy "refresh.png" "%dir_name%"
copy "scan.png" "%dir_name%"
copy "close.png" "%dir_name%"

REM Удаляем временную папку dist
rmdir /s /q dist

REM Удаляем папку __pycache__
rmdir /s /q __pycache__

echo.
echo Сборка завершена. Итоговый исполняемый файл и дополнительные файлы находятся в папке: %dir_name%
pause