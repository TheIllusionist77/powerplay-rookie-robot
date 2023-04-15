import gspread
import pandas as pd
from oauth2client.service_account import ServiceAccountCredentials

scope = ["https://www.googleapis.com/auth/spreadsheets", "https://www.googleapis.com/auth/drive"]
creds = ServiceAccountCredentials.from_json_keyfile_name("ftc-robotics-383722-7147c409ec52.json", scope)
client = gspread.authorize(creds)

sheet = client.open("Matches")
sheet_instance = sheet.get_worksheet(0)

counter = 0

query = int(input("Enter team number to search up: "))

data_dict = {}
for team_data in sheet_instance.get_all_records():
    if int(team_data["Team Number"]) == query:
        for i in team_data:
            print("{:<15}".format(i), team_data[i])