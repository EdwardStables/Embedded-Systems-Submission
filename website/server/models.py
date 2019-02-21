from django.db import models

class sensor_values(models.Model):
    time = models.DateTimeField() 
    temp_discrete = models.DecimalField(decimal_places=1, max_digits=4) 
    temp_integrated = models.DecimalField(decimal_places=1, max_digits=4)
    gyro_x = models.IntegerField() 
    gyro_y = models.IntegerField()
    gyro_z = models.IntegerField()
    accel_x = models.DecimalField(decimal_places=3, max_digits=4)
    accel_y = models.DecimalField(decimal_places=3, max_digits=4)
    accel_z = models.DecimalField(decimal_places=3, max_digits=4)
    angle = models.IntegerField()

class violation_values(models.Model):
    time = models.DateTimeField() 
    angle = models.IntegerField()
    accel = models.DecimalField(decimal_places = 1, max_digits=4) 
    temp = models.DecimalField(decimal_places=1, max_digits=4) 

class sensor_limits(models.Model):
    current = models.IntegerField()
    Max_Angle = models.IntegerField()
    Max_Discrete_Temperature = models.IntegerField()
    Min_Discrete_Temperature = models.IntegerField()
    Max_Integrated_Temperature = models.IntegerField()
    Min_Integrated_Temperature = models.IntegerField()
    Max_Acceleration = models.DecimalField(max_digits=2, decimal_places=1)
