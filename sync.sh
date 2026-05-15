#!/bin/bash

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Get current branch name automatically
BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo -e "${YELLOW}>>> Working on branch: ${BRANCH}${NC}"

# 1. Cleanup
if [ -f "nul" ]; then rm -f "nul"; fi

# 2. Check for changes
if [ -z "$(git status --porcelain)" ]; then
    echo -e "${GREEN}Everything is up to date. No changes to sync.${NC}"
    sleep 2
    exit
fi

# 3. Pull first to avoid conflicts
echo -e "${YELLOW}>>> Pulling latest changes from remote...${NC}"
git pull origin $BRANCH

# 4. Add and Commit
git add .
echo -e "${YELLOW}Enter commit message (Press Enter for default):${NC}"
read msg
if [ -z "$msg" ]; then
    msg="Update: $(date +'%Y-%m-%d %H:%M:%S')"
fi
git commit -m "$msg"

# 5. Push
echo -e "${YELLOW}>>> Pushing to GitHub and Gitee...${NC}"
if git push origin $BRANCH; then
    echo -e "${GREEN}====================================${NC}"
    echo -e "${GREEN}      SYNC SUCCESSFUL!              ${NC}"
    echo -e "${GREEN}====================================${NC}"
else
    echo -e "${RED}      SYNC FAILED! Check conflicts. ${NC}"
fi

sleep 3