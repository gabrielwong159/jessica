from telegram.ext import Updater

users = {
    # 'yos': 183753003,
    'gabriel': 218740835,
}

with open('token', 'r') as f:
    token = f.read().strip()
updater = Updater(token=token)
bot = updater.dispatcher.bot


def send_message(message):
    for chat_id in users.values():
        bot.send_message(chat_id=chat_id, text=message)

