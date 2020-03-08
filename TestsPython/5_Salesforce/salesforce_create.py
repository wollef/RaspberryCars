from simple_salesforce import Salesforce

sf = Salesforce(
    username='thibault.leblond@wollef.org',
    password='fjamaw;-)1',
    security_token='uEGXRLdynZdEtd8Kef2mZANC'
);

sf.Car__c.create({'Name':'My Raspberry Car !!!', 'Car_number__c':'RC 1234'})
