#!/bin/bash

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}>>> Starting code sync to GitHub and Gitee...${NC}"

# 1. Clean up invalid system filenames (Windows specific)
if [ -f "nul" ]; then
    rm -f "nul"
fi

# 2. Check for changes
if [ -z "$(git status --porcelain)" ]; then
    echo -e "${GREEN}No changes detected. Nothing to sync.${NC}"
    sleep 2
    exit
fi

# 3. Add changes to staging area
if git add .; then
    echo -e "${GREEN}Changes added to staging area successfully.${NC}"
else
    echo -e "${RED}Failed to add files! Check for invalid filenames.${NC}"
    read -p "Press Enter to exit..."
    exit 1
fi

# 4. Prompt for commit message
echo -e "${YELLOW}Enter commit message (Press Enter for default):${NC}"
read msg

# Use default message if input is empty
if [ -z "$msg" ]; then
    msg="Update: $(date +'%Y-%m-%d %H:%M:%S')"
fi

# 5. Commit changes
git commit -m "$msg"

# 6. Push to remotes
echo -e "${YELLOW}Pushing to remote repositories...${NC}"
if git push origin master; then
    echo -e "${GREEN}====================================${NC}"
    echo -e "${GREEN}      SYNC SUCCESSFUL!              ${NC}"
    echo -e "${GREEN}  Code uploaded to GitHub & Gitee   ${NC}"
    echo -e "${GREEN}====================================${NC}"
else
    echo -e "${RED}====================================${NC}"
    echo -e "${RED}      SYNC FAILED!                  ${NC}"
    echo -e "${RED}  Check network or SSH config       ${NC}"
    echo -e "${RED}====================================${NC}"
fi

# Exit countdown
echo -e "${YELLOW}Closing in 3 seconds...${NC}"
sleep 3