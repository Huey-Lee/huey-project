#!/bin/bash

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${YELLOW}>>> 开始同步代码到 GitHub 和 Gitee...${NC}"

# 1. 尝试删除可能存在的非法 nul 文件
if [ -f "nul" ]; then
    rm -f "nul"
fi

# 2. 检查是否有文件变动
if [ -z "$(git status --porcelain)" ]; then
    echo -e "${GREEN}没有任何文件修改，无需同步。${NC}"
    sleep 2
    exit
fi

# 3. 添加更改到暂存区并检查是否成功
if git add .; then
    echo -e "${GREEN}已成功添加更改到暂存区${NC}"
else
    echo -e "${RED}添加文件失败！请检查是否有非法文件名（如 nul）。${NC}"
    pause
    exit 1
fi

# 4. 让用户输入备注
echo -e "${YELLOW}请输入修改备注 (直接回车将使用默认备注):${NC}"
read msg
if [ -z "$msg" ]; then
    msg="Update: $(date +'%Y-%m-%d %H:%M:%S')"
fi

# 5. 提交
git commit -m "$msg"

# 6. 推送
echo -e "${YELLOW}正在推送到双平台...${NC}"
if git push origin master; then
    echo -e "${GREEN}====================================${NC}"
    echo -e "${GREEN}   同步成功！代码已飞向 GitHub/Gitee ${NC}"
    echo -e "${GREEN}====================================${NC}"
else
    echo -e "${RED}同步失败，请检查网络或 SSH 配置！${NC}"
fi

echo -e "${YELLOW}窗口将在 3 秒后自动关闭...${NC}"
sleep 3