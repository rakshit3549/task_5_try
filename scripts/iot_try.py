import requests

# defining our sheet name in the 'id' variable and the the column where we want to update the value
parameters = {'City':'Mumbai','Team Id':'205','Shipped Quantity':'1',"id":"OrdersShipped","Priority":"LP",'Estimated Time of Delivery': '2000','Unique Id':'Pandemic','Shipped Status':'YES','Item':'Clothes','Cost':'150','Shipped Date and Time':'02/07/21 22:02:29','Order ID':'3001'}

parameter = {'Shipped Quantity':'1','City':'Mumbai','Team Id':'205','Estimated Time of Delivery':'2000','Unique Id':'Pandemic','Shipped Status':'YES','Priority': 'LP','Item':'Clothes','Cost':'150','Shipped Date and Time':'02/07/21 22:02:29','id':'OrdersShipped','Order ID':'3001'}

URL = "https://script.google.com/macros/s/AKfycbxzDT9V-wHKWMmVTeVw4HEksTDW5CUCDanxoKQbHk0RUHScBaE5kvYq/exec"

response = requests.get(URL, params=parameters)

print(response.content)
