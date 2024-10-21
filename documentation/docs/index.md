# Home

This is a library for "ticking" statemachines.
Each [TickingState][yasmin_etasl.yasmin_ticking.TickingState] (see [API][yasmin_etasl.yasmin_ticking.TickingState] for detailed definition) is defined by the following methods:

 - **entry(self,blackboard)**:
    executed when the state is entered
    
 - **doo(self,blackboard)**:
    execute while the state is running.  The state can take a longer time
    but should regularly yield by returning TICKING.
 - **exit(self)**:
    is execute when the state exits, note that it does not has the blackboard as argument.
    Will even be called when the other methods return an exception.
 - **reset(self)**:
    Resets the state (i.e. calls exit() when appropriate and ensures that the next time, entry() will be called)


When using the state one calls it with the () operator. This calls then the execute() method that calls the entry,doo,exit methods
appropriately.

The [visitor pattern](https://en.wikipedia.org/wiki/Visitor_pattern) is used to be able to generically travers the hierarchy of states.
the **accept** method of a TickingState calls the visitor appropriately.  Visitor is defined  [here][yasmin_etasl.yasmin_ticking.Visitor]


The [Generator][yasmin_etasl.yasmin_ticking.Generator] class uses a python generator to define a TickingState. 
This allows to define TickingStates using a python generator method `co_execute(self, blackboard:Blackboard)`.  This routine has to regularly 
yield control using a `yield <outcom>` statement. This makes it easy to specify a TickingState.

