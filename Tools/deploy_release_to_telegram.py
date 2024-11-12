#!/usr/bin/env python3
"""
Deploy release binaries to telegram
"""
import os
import logging
import argparse
import subprocess
import requests
from typing import Optional

logger = logging.getLogger(__name__)

REQUIRED_FILES = 6

def get_git_info():
    commit_sha = subprocess.check_output(['git', 'rev-parse', '--short=8', 'HEAD']).decode('utf-8').strip()
    commit_date = subprocess.check_output(['git', 'log', '-1', '--format=%cd', '--date=short']).decode('utf-8').strip()
    committer_name = subprocess.check_output(['git', 'log', '-1', '--format=%an']).decode('utf-8').strip()
    committer_email = subprocess.check_output(['git', 'log', '-1', '--format=%ae']).decode('utf-8').strip()
    return commit_sha, commit_date, committer_name, committer_email

def find_files(dir) -> Optional[list]:
    files = sorted([os.path.join(root, f) for root, dirs, files in os.walk(dir) for f in files if f.endswith('.px4')])
    if len(files) < REQUIRED_FILES:
        logger.error(f"Need at least {REQUIRED_FILES} .px4 files to send. Found {len(files)}.")
        return

    logger.info(f"Found {len(files)} .px4 files.")

    return files

def create_text_message() -> str:
    commit_sha, commit_date, committer_name, committer_email = get_git_info()
    return (
        "Px4 Autopilot\n"
        f"VCS commit: {commit_sha}\n"
        f"Commit date: {commit_date}\n"
        f"Author: {committer_name} <{committer_email}>"
    )

def send_media_group(telegram_bot_token, telegram_chat_id, files, caption):
    media_json_array = []
    for i in range(1, REQUIRED_FILES):
        file_entry = {"type": "document", "media": f"attach://file{i}"}
        media_json_array.append(file_entry)

    last_file_enty = {"type": "document", "media": f"attach://file{REQUIRED_FILES}", "caption": caption}
    media_json_array.append(last_file_enty)

    media_payload = {
        'chat_id': telegram_chat_id,
        'media': str(media_json_array).replace("'", '"')
    }

    files_payload = {}
    for idx in range(0, REQUIRED_FILES):
        files_payload[f"file{idx + 1}"] = open(files[idx], 'rb')

    response = requests.post(f'https://api.telegram.org/bot{telegram_bot_token}/sendMediaGroup', data=media_payload, files=files_payload)

    for file in files_payload.values():
        file.close()

    return response.json()

def main():
    parser = argparse.ArgumentParser(description="Send media group via Telegram bot.")
    parser.add_argument('--bot-token', required=True, help='Telegram bot token')
    parser.add_argument('--chat-id', required=True, help='Telegram chat ID')
    parser.add_argument('--input-dir', default='build/')
    args = parser.parse_args()

    files = find_files(dir=args.input_dir)
    if files is None:
        logger.error(f"No .px4 files found in {args.input_dir}.")
        os._exit(1)

    text_message = create_text_message()

    res = send_media_group(args.bot_token, args.chat_id, files, text_message)
    logger.info(res)

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    logger.setLevel(logging.INFO)

    main()
