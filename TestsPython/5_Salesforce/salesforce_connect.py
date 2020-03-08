from simple_salesforce import Salesforce

sf = Salesforce(
    username='thibault.leblond@wollef.org',
    password='fjamaw;-)1',
    security_token='uEGXRLdynZdEtd8Kef2mZANC'
);

response= sf.query_all("SELECT Id, Name, Car_number__c FROM Car__c ");

cars = response['records']

for car in cars:
   print("ID",   car["Id"] );
   print("Name", car["Name"] );
   print("Number", car["Car_number__c"] );
   print('');
