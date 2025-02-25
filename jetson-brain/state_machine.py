from transitions import Machine

class RobotStateMachine:
    def __init__(self):
        states = [
            'IDLE', 
            'IDENTIFY', 
            'TO_OBJECT', 
            'PICK', 
            'TO_TIDY_DESTINATION', 
            'PLACE_OBJECT', 
            'REVIEW'
        ]
        
        self.machine = Machine(model=self, states=states, initial='IDLE')

        # Define transitions
        self.machine.add_transition('detect_objects', 'IDLE', 'IDENTIFY')
        self.machine.add_transition('plan_path_to_object', 'IDENTIFY', 'TO_OBJECT')
        self.machine.add_transition('pick_object', 'TO_OBJECT', 'PICK')
        self.machine.add_transition('plan_path_to_destination', 'PICK', 'TO_TIDY_DESTINATION')
        self.machine.add_transition('place_object', 'TO_TIDY_DESTINATION', 'PLACE_OBJECT')
        self.machine.add_transition('plan_next_object', 'PLACE_OBJECT', 'TO_OBJECT')

        self.machine.add_transition('review_objects', 'PLACE_OBJECT', 'REVIEW')

    