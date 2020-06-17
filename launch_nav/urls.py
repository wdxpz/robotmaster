from django.conf.urls import url

from views import index, reset

urlpatterns = [
    # url(r'^(.+)/$', index, name='index')
    url(r'^$', index, name='index'),
    url(r'^reset/$', reset, name='reset')
]