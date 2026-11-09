@echo off
chcp 65001 > nul

set /p commit_message="コミットメッセージを入力してください: "

if "%commit_message%"=="" (
    echo メッセージが空のため、処理を中止しました。
    pause
    exit /b
)

git add .
git commit -m "%commit_message%"
git push

echo.
echo すべての処理が完了しました。
pause
