# (once((historically[0:3]{altitude >= 0.3, altitude<=0.7}) and (once[3:23]{takeoff})))


import oracle

# property to verify
PROPERTY = "(once(historically[0:10]{altitude >= 0.3, altitude<=0.7}))"

# predicates used in the property (initialization for time 0)
predicates = dict(
    time = 0,
    altitude=0.0
)
# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    if message['topic'] == 'mavros/local_position/pose':
        predicates['altitude'] = message['pose']['position']['z']
    predicates['time'] = message['time']
    return predicates
# This function has to be defined by the user depending on the property defined.
# In this case we have just implemented a simple and general function which
# updates the predicates if it finds the topic in the list of predicates.
# Since the property is defined on predicates, we need this function to update the
# predicates each time a message is observed. This abstraction of course is totally
# dependent on the specific application.
