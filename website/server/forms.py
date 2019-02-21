from django import forms

class LimitForm(forms.Form):
    min_temp = forms.IntegerField(label = 'Minimum Temperature', max_value=100, min_value=-30)
    max_temp = forms.IntegerField(label = 'Maximum Temperature', max_value=100, min_value=-30)
    max_angle = forms.IntegerField(label = 'Maximum Angle', min_value=1, max_value=180)
    max_accel = forms.DecimalField(label = 'Maximum Acceleration', max_value=2, max_digits=2, min_value=0.1)