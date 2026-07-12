//! Flight mode state machine (`spec.txt` §6.1).
//!
//! A small, explicit FSM guarding mode transitions. Invalid transitions are
//! rejected (returns `None`), so the autopilot can never silently enter an
//! unsafe mode.

/// High-level flight modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    Disarmed,
    Armed,
    Takeoff,
    PositionHold,
    Land,
    Failsafe,
    Glide,
}

/// Events that drive mode transitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightEvent {
    Arm,
    Disarm,
    CommandTakeoff,
    ReachedTargetAltitude,
    CommandLand,
    OnGround,
    Fault,
    Recovered,
    PropulsionLoss,
}

/// Explicit flight state machine.
#[derive(Debug, Clone, Copy)]
pub struct FlightStateMachine {
    mode: FlightMode,
}

impl FlightStateMachine {
    pub const fn new() -> Self {
        Self {
            mode: FlightMode::Disarmed,
        }
    }

    pub const fn mode(&self) -> FlightMode {
        self.mode
    }

    /// Attempt a transition. Returns the new mode, or `None` if the event is
    /// not valid in the current mode.
    pub fn handle(&mut self, ev: FlightEvent) -> Option<FlightMode> {
        let next = match (self.mode, ev) {
            (FlightMode::Disarmed, FlightEvent::Arm) => FlightMode::Armed,
            (FlightMode::Armed, FlightEvent::CommandTakeoff) => FlightMode::Takeoff,
            (FlightMode::Takeoff, FlightEvent::ReachedTargetAltitude) => FlightMode::PositionHold,
            (FlightMode::PositionHold, FlightEvent::CommandLand) => FlightMode::Land,
            (FlightMode::Land, FlightEvent::OnGround) => FlightMode::Disarmed,
            // Faults dominate: anything flight-capable falls back to Failsafe.
            (m, FlightEvent::Fault) if m != FlightMode::Disarmed => FlightMode::Failsafe,
            (FlightMode::Failsafe, FlightEvent::Recovered) => FlightMode::Armed,
            (FlightMode::Armed, FlightEvent::Disarm) => FlightMode::Disarmed,
            (FlightMode::Failsafe, FlightEvent::Disarm) => FlightMode::Disarmed,
            // Total propulsion loss: enter engine-out glide from any powered
            // (or landing) mode. Glide itself is terminal for this event.
            (m, FlightEvent::PropulsionLoss)
                if m != FlightMode::Disarmed && m != FlightMode::Glide =>
            {
                FlightMode::Glide
            }
            (FlightMode::Glide, FlightEvent::CommandLand) => FlightMode::Land,
            (FlightMode::Glide, FlightEvent::OnGround) => FlightMode::Disarmed,
            _ => return None,
        };
        self.mode = next;
        Some(next)
    }

    /// Whether the vehicle is currently expected to be airborne / powered.
    pub const fn is_flight_capable(&self) -> bool {
        matches!(
            self.mode,
            FlightMode::Takeoff
                | FlightMode::PositionHold
                | FlightMode::Land
                | FlightMode::Failsafe
                | FlightMode::Glide
        )
    }
}

impl Default for FlightStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_sequence() {
        let mut fsm = FlightStateMachine::new();
        assert_eq!(fsm.handle(FlightEvent::Arm), Some(FlightMode::Armed));
        assert_eq!(
            fsm.handle(FlightEvent::CommandTakeoff),
            Some(FlightMode::Takeoff)
        );
        assert_eq!(
            fsm.handle(FlightEvent::ReachedTargetAltitude),
            Some(FlightMode::PositionHold)
        );
        assert_eq!(fsm.handle(FlightEvent::CommandLand), Some(FlightMode::Land));
        assert_eq!(
            fsm.handle(FlightEvent::OnGround),
            Some(FlightMode::Disarmed)
        );
    }

    #[test]
    fn fault_overrides() {
        let mut fsm = FlightStateMachine::new();
        fsm.handle(FlightEvent::Arm);
        fsm.handle(FlightEvent::CommandTakeoff);
        assert_eq!(fsm.handle(FlightEvent::Fault), Some(FlightMode::Failsafe));
        assert!(fsm.is_flight_capable());
        assert_eq!(fsm.handle(FlightEvent::Recovered), Some(FlightMode::Armed));
    }

    #[test]
    fn invalid_transition_rejected() {
        let mut fsm = FlightStateMachine::new();
        // Cannot take off while disarmed.
        assert_eq!(fsm.handle(FlightEvent::CommandTakeoff), None);
    }
}
