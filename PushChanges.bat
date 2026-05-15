@echo off
:: 设置字符集为 UTF-8，防止中文乱码
chcp 65001 > nul

git add .
set /p msg="请输入修改备注: "
git commit -m "%msg%"
git push

echo.
echo ============================
echo 同步完成！
echo ============================
pause