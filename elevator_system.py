from collections import namedtuple
from enum import Enum
import heapq


Request = namedtuple("Request", ("floor", "direction"))


class Direction(Enum):
    UP = 1
    DOWN = 2
    IDLE = 3


class ElevatorSystem:
    def __init__(self) -> None:
        self.elevators = []
        self.floors = []

    def assign(self, request):
        # determines most optimal elevator
        elevator = self.pick_elevator(request)

        elevator.accept_request(request.floor)

    # Could be extracted into a new ElevatorAssigner class
    def pick_elevator(self, request):
        # picks that closest elevator that's either idle or moving in the requested floor's direction
        # if all elevators are moving in the wrong direction, pick the one that's closest when the request is received

        def wrong_direction(elevator):
            # Elevator and request going in different direction
            if elevator.direction != Direction.IDLE and elevator.direction != request.direction:
                return True

            # Elevator is above requested floor and moving up (away from requested floor)
            if elevator.current_floor > request.floor and elevator.direction == Direction.UP:
                return True

            # Elevator is below requested floor and moving down (away from requested floor)
            if elevator.current_floor < request.floor and elevator.direction == Direction.DOWN:
                return True

            return False

        closest_elevator, closest_wrong_elevator = None, None
        closest_distance, closest_wrong_distance = float('inf'), float('inf')

        for elevator in self.elevators:
            distance = abs(elevator.current_floor - request.floor)

            if wrong_direction(elevator):
                if distance < closest_wrong_distance:
                    closest_wrong_distance = distance
                    closest_wrong_elevator = elevator
            else:
                if distance < closest_distance:
                    closest_distance = distance
                    closest_elevator = elevator

        return closest_elevator if closest_elevator else closest_wrong_elevator

    def move_elevators(self):
        # following loop doesn't work in a single threaded language like Python
        # it's meant to approximate the real system which will have multiple processes
        # and can have one process/thread can this loop indefinitely
        while True:
            for elevator in self.elevators:
                if elevator.can_move():
                    elevator.move()


class Elevator:
    def __init__(self) -> None:
        self.current_floor = 0
        self.direction = Direction.IDLE
        self.floor_pad = FloorPad(self)

        self.up_requests = []
        self.down_requests = []

        heapq.heapify(self.up_requests)
        heapq.heapify(self.down_requests)

    def move(self):
        self.update_direction()

        if self.direction == Direction.UP:
            self.floor = heapq.heappop(self.up_requests)
        elif self.direction == Direction.DOWN:
            self.floor = heapq.heappop(self.down_requests)

        # insert wait command so that it takes time to move floors

        self.update_direction()

    def can_move(self):
        return len(self.up_requests) + len(self.down_requests) > 0

    def update_direction(self):
        if len(self.up_requests) + len(self.down_requests) == 0:
            self.direction = Direction.IDLE

        # Change directions if necessary
        elif self.direction == Direction.UP and len(self.up_requests) == 0:
            self.direction == Direction.DOWN

        elif self.direction == Direction.DOWN and len(self.down_requests) == 0:
            self.direction = Direction.UP

    def accept_request(self, floor):
        # assume no duplcate floors
        # could prevent duplicate entries by maintaining a set of active floor requests

        if floor > self.current_floor:
            heapq.heappush(self.up_requests, floor)
        else:
            heapq.heappush(self.down_requests, floor)


class FloorPad:
    def __init__(self, elevator) -> None:
        self.elevator = elevator

    def press_button(self, floor):
        self.elevator.accept_request(floor)


class Floor:
    def __init__(self, floor, elevator_system, up_button=True, down_button=True) -> None:
        self.floor = floor
        self.elevator_system = elevator_system

        self.up_button = up_button
        self.down_button = down_button

    def request(self, direction):
        def invalid_direction():
            return direction == Direction.UP and not self.up_button or \
                direction == Direction.DOWN and not self.down_button

        # handle edge floors that can only move in one direction
        if invalid_direction():
            # throw error
            return

        self.elevator_system.assign(Request(self.floor, direction))
