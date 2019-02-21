from django.shortcuts import render, HttpResponseRedirect
from django.urls import reverse
import json
import datetime
from .mqtt import publish_client
from .models import sensor_limits, sensor_values, violation_values
from .forms import LimitForm
# Create your views here.


#################################################################################
# this handles the serving of data to the html template, as well as sending     #
# messages to the pi via mqtt and getting user input.                           #
#################################################################################

def profile(request):
    if request.method == "POST":
      form = LimitForm(request.POST)
      if form.is_valid():
        current_limits = sensor_limits.objects.get(current = 1)

        different = False
        
        if current_limits.Max_Angle != form.cleaned_data['max_angle']:
          current_limits.Max_Angle = form.cleaned_data['max_angle']
          different = True
        if current_limits.Max_Discrete_Temperature != form.cleaned_data['max_temp']:
          current_limits.Max_Discrete_Temperature = form.cleaned_data['max_temp']
          different = True
        if current_limits.Min_Discrete_Temperature != form.cleaned_data['min_temp']:
          current_limits.Min_Discrete_Temperature = form.cleaned_data['min_temp']
          different = True
        if current_limits.Max_Acceleration != form.cleaned_data['max_accel']:
          current_limits.Max_Acceleration = form.cleaned_data['max_accel']
          different = True

        if different:
          current_limits.save()
          transmission_data = {
          'Reset' : True,
          'Max_Angle' : form.cleaned_data['max_angle'],
          'Max_Discrete_Temperature' : form.cleaned_data['max_temp'],
          'Min_Discrete_Temperature' : form.cleaned_data['min_temp'],
          'Max_Acceleration' : float(form.cleaned_data['max_accel'])
          }
          transmission_data = json.dumps(transmission_data)
          send_client = publish_client('web_message', ip = 'test.mosquitto.org', port = 8884, encrypt = True)
          send_client.publish('IC.embedded/jaffa_cakes/comms/server', transmission_data)
          send_client.close_connection()
        return HttpResponseRedirect(reverse('profile'))

      if 'Reset' in request.POST:
        print('reset')
        transmission_data = json.dumps({'Reset' : True})
        send_client = publish_client('web_message', ip = 'test.mosquitto.org', port = 8884, encrypt = True)
        send_client.publish('IC.embedded/jaffa_cakes/comms/server', transmission_data)
        send_client.close_connection()    
        return HttpResponseRedirect(reverse('profile'))

    current_limits = sensor_limits.objects.get(current = 1)
    form = LimitForm(initial = {
      'min_temp' : int(current_limits.Min_Discrete_Temperature),
      'max_temp' : int(current_limits.Max_Discrete_Temperature),
      'max_angle' : int(current_limits.Max_Angle),
      'max_accel' : float(current_limits.Max_Acceleration)
    })
    
    start_date = datetime.datetime.now() - datetime.timedelta(minutes = 10)
    time_vals = sensor_values.objects.filter(time__gte=start_date).values('time')
    temp_vals = sensor_values.objects.filter(time__gte=start_date).values('temp_discrete')
    angle_vals = sensor_values.objects.filter(time__gte=start_date).values('angle')
    violations = list(violation_values.objects.filter(time__gte=start_date))[::-1]
    
    context = {
      'time_data' : [str(a['time']) for a in list(time_vals)],
      'temp_data' : [float(a['temp_discrete']) for a in list(temp_vals)],
      'angle_data' : [a['angle'] for a in list(angle_vals)],
      'violations' : violations,
      'form' : form
    }
    
    return render(request, 'server/index.html', context)