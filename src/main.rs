use rand::prelude::*;
use std::vec;

const QT_CAPACITY: usize = 3;
const WORLD_WIDTH: Coord = 800.0;
const WORLD_HEIGHT: Coord = 800.0;
const WORLD_CENTER: Vector2D = Vector2D {
    x: WORLD_WIDTH / 2.0,
    y: WORLD_HEIGHT / 2.0,
};

type Coord = f32;

#[derive(Debug)]
struct Subdivision {
    ne: Box<Quadtree>,
    nw: Box<Quadtree>,
    se: Box<Quadtree>,
    sw: Box<Quadtree>,
}

type BoidID = usize;

#[derive(Debug, Clone, Copy)]
struct Vector2D {
    x: Coord,
    y: Coord,
}

impl Vector2D {
    fn new(x: Coord, y: Coord) -> Vector2D {
        Vector2D { x, y }
    }

    fn sq_distance_to(&self, other: &Vector2D) -> Coord {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    fn mag_sq(&self) -> Coord {
        self.x * self.x + self.y * self.y
    }

    fn mag(&self) -> Coord {
        self.mag_sq().sqrt()
    }

    fn add(&mut self, other: &Vector2D) -> &mut Self {
        self.x += other.x;
        self.y += other.y;
        self
    }

    fn add_scaled(&mut self, other: &Vector2D, scalar: Coord) -> &mut Self {
        self.x += other.x * scalar;
        self.y += other.y * scalar;
        self
    }

    fn sub(&mut self, other: &Vector2D) {
        self.x -= other.x;
        self.y -= other.y;
    }

    fn normalize(&mut self) -> &mut Self {
        let mag = self.mag();
        self.x /= mag;
        self.y /= mag;
        self
    }

    fn clamp(&mut self, min: Coord, max: Coord) -> &mut Self {
        debug_assert!(min <= max);
        debug_assert!(0.0 <= min);
        debug_assert!(0.0 <= max);

        let mag = self.mag();
        if mag < min {
            self.normalize().mult_scalar(min);
        } else if mag > max {
            self.normalize().mult_scalar(max);
        }
        self
    }

    fn mult_scalar(&mut self, scalar: Coord) -> &mut Self {
        self.x *= scalar;
        self.y *= scalar;
        self
    }

    fn div_scalar(&mut self, scalar: Coord) -> &mut Self {
        self.x /= scalar;
        self.y /= scalar;
        self
    }
}

#[derive(Debug)]
struct BoidPool {
    positions: Vec<Vector2D>,
    current_headings: Vec<Vector2D>,
    calculated_headings: Vec<Vector2D>,
}

impl BoidPool {
    fn new() -> BoidPool {
        BoidPool {
            positions: vec![],
            current_headings: vec![],
            calculated_headings: vec![],
        }
    }

    fn push(&mut self, pos_x: Coord, pos_y: Coord, heading_x: Coord, heading_y: Coord) {
        self.positions.push(Vector2D::new(pos_x, pos_y));
        self.current_headings
            .push(Vector2D::new(heading_x, heading_y));
        self.calculated_headings.push(Vector2D::new(0.0, 0.0));
    }

    fn get_position(&self, id: BoidID) -> &Vector2D {
        &self.positions[id]
    }

    fn set_position(&mut self, id: BoidID, x: Coord, y: Coord) {
        self.positions[id].x = x;
        self.positions[id].y = y;
    }

    fn get_heading(&self, id: BoidID) -> &Vector2D {
        &self.current_headings[id]
    }

    fn set_heading(&mut self, id: BoidID, x: Coord, y: Coord) {
        self.current_headings[id].x = x;
        self.current_headings[id].y = y;
    }

    fn get_future_heading(&mut self, id: BoidID) -> &Vector2D {
        &self.current_headings[id]
    }

    fn future_heading_add_scaled(&mut self, id: BoidID, v: &Vector2D, scalar: Coord) {
        self.calculated_headings[id].add_scaled(v, scalar);
    }

    fn future_heading_set(&mut self, id: BoidID, x: Coord, y: Coord) {
        self.calculated_headings[id].x = x;
        self.calculated_headings[id].y = y;
    }

    fn len(&self) -> usize {
        self.positions.len()
    }
}

#[derive(Debug)]
struct RectContainer {
    lx: Coord,
    rx: Coord,
    ty: Coord,
    by: Coord,
}

impl RectContainer {
    fn new(lx: Coord, rx: Coord, ty: Coord, by: Coord) -> RectContainer {
        RectContainer { lx, rx, ty, by }
    }

    fn new_from_center(cx: Coord, cy: Coord, width: Coord, height: Coord) -> RectContainer {
        let half_width = width / 2.0;
        let half_height = height / 2.0;
        RectContainer {
            lx: cx - half_width,
            rx: cx + half_width,
            ty: cy - half_height,
            by: cy + half_height,
        }
    }

    fn contains(&self, x: Coord, y: Coord) -> bool {
        let x_inside = self.lx <= x && x <= self.rx;
        let y_inside = self.ty <= y && y <= self.by;
        x_inside && y_inside
    }

    fn intersects(&self, other: &RectContainer) -> bool {
        let no_x_overlap = self.rx < other.lx || other.rx < self.lx;
        let no_y_overlap = self.by < other.ty || other.by < self.ty;

        !(no_x_overlap || no_y_overlap)
    }
}

#[derive(Debug)]
struct Quadtree {
    container: RectContainer,
    boids: Vec<BoidID>,
    subdivisions: Option<Subdivision>,
}

impl Quadtree {
    fn new(lx: Coord, rx: Coord, ty: Coord, by: Coord) -> Quadtree {
        Quadtree {
            container: RectContainer::new(lx, rx, ty, by),
            boids: Vec::with_capacity(QT_CAPACITY),
            subdivisions: None,
        }
    }

    fn push(&mut self, boid_id: BoidID, bp: &BoidPool) -> Result<(), String> {
        // Safety check to see if this is the correct quadtree to add to.
        let position = bp.get_position(boid_id);
        if !self.container.contains(position.x, position.y) {
            return Err(format!(
                "The boid with id {} in position `{:?}` cannot go in the quadtree bounded by `{:?}`.",
                boid_id, position, &self.container
            ));
        }

        self.push_unchecked_with_position(boid_id, bp, position)
    }

    fn push_unchecked_with_position(
        &mut self,
        boid_id: BoidID,
        bp: &BoidPool,
        position: &Vector2D,
    ) -> Result<(), String> {
        if self.subdivisions.is_none() {
            if self.boids.len() < QT_CAPACITY {
                // If there is still room, push here.
                self.boids.push(boid_id);
                return Ok(());
            } else {
                // Else, there's not enough room, so we must subdivide if we haven't already.
                self.subdivide(bp);
            }
        }

        // Then add to the matching child.
        let quad = self.find_which_child(position);

        let sub = match self.subdivisions {
            Some(ref mut sub) => sub,
            None => {
                unreachable!()
            }
        };

        match quad {
            Quadrant::NE => sub.ne.push(boid_id, bp),
            Quadrant::NW => sub.nw.push(boid_id, bp),
            Quadrant::SE => sub.se.push(boid_id, bp),
            Quadrant::SW => sub.sw.push(boid_id, bp),
        }
    }

    fn subdivide(&mut self, bp: &BoidPool) {
        let cx = (self.container.lx + self.container.rx) / 2.0;
        let cy = (self.container.ty + self.container.by) / 2.0;
        let ne = Box::new(Quadtree::new(cx, self.container.rx, self.container.ty, cy));
        let nw = Box::new(Quadtree::new(self.container.lx, cx, self.container.ty, cy));
        let sw = Box::new(Quadtree::new(self.container.lx, cx, cy, self.container.by));
        let se = Box::new(Quadtree::new(cx, self.container.rx, cy, self.container.by));

        self.subdivisions = Some(Subdivision { ne, nw, se, sw });

        let boids_ids = std::mem::take(&mut self.boids);

        for &boid_id in boids_ids.iter() {
            let position = bp.get_position(boid_id);
            let quad = self.find_which_child(position);

            let sub = match self.subdivisions {
                Some(ref mut sub) => sub,
                None => {
                    unreachable!()
                }
            };

            let r = match quad {
                Quadrant::NE => sub.ne.push(boid_id, bp),
                Quadrant::NW => sub.nw.push(boid_id, bp),
                Quadrant::SE => sub.se.push(boid_id, bp),
                Quadrant::SW => sub.sw.push(boid_id, bp),
            };
            r.expect("It should be able to push to the children.");
        }
    }

    fn find_which_child(&self, position: &Vector2D) -> Quadrant {
        let cx = (self.container.lx + self.container.rx) / 2.0;
        let cy = (self.container.ty + self.container.by) / 2.0;
        let (hor, ver) = (position.x <= cx, position.y <= cy);
        match (hor, ver) {
            (true, true) => Quadrant::NW,
            (true, false) => Quadrant::SW,
            (false, true) => Quadrant::NE,
            (false, false) => Quadrant::SE,
        }
    }

    fn rebalance(&mut self, bp: &BoidPool) {
        self._rebalance(bp, None)
    }

    fn _rebalance(&mut self, bp: &BoidPool, parent: Option<&mut Quadtree>) {
        // Go deep first and throw the bad ones up.
        if let Some(mut sub) = self.subdivisions.take() {
            sub.ne._rebalance(bp, Some(self));
            sub.nw._rebalance(bp, Some(self));
            sub.se._rebalance(bp, Some(self));
            sub.sw._rebalance(bp, Some(self));

            self.subdivisions = Some(sub);
        }

        // Bubble up the ones that have become misplaced.
        if let Some(parent) = parent {
            self.boids.retain(|&id| {
                let position = bp.get_position(id);
                if self.container.contains(position.x, position.y) {
                    true
                } else {
                    println!(
                        "Bubbling boid with id {} in position `{:?}` up from {:?}",
                        id, position, self.container
                    );
                    match parent.push_unchecked_with_position(id, bp, position) {
                        Ok(_) => false,
                        Err(e) => {
                            panic!("Error while rebalancing: {}", e);
                        }
                    }
                }
            });
        }
        // If no parent, do nothing, since we must be at the top-most level.
    }

    fn query_range(&self, query: &RectContainer, bp: &BoidPool) -> Vec<BoidID> {
        let mut results = vec![];

        if !self.container.intersects(query) {
            return results;
        }

        for &id in self.boids.iter() {
            let position = bp.get_position(id);
            if query.contains(position.x, position.y) {
                results.push(id);
            }
        }

        if let Some(ref sub) = self.subdivisions {
            results.append(&mut sub.ne.query_range(query, bp));
            results.append(&mut sub.nw.query_range(query, bp));
            results.append(&mut sub.se.query_range(query, bp));
            results.append(&mut sub.sw.query_range(query, bp));
        }
        results
    }
}

enum Quadrant {
    NE,
    NW,
    SE,
    SW,
}

#[derive(Debug)]
struct BoidWorld {
    qt: Quadtree,
    bp: BoidPool,
}

impl BoidWorld {
    fn new(width: Coord, height: Coord) -> BoidWorld {
        BoidWorld {
            qt: Quadtree::new(0.0, width * 4.0, 0.0, height * 4.0),
            bp: BoidPool::new(),
        }
    }

    fn populate<T: Rng>(&mut self, quantity: usize, rng: &mut T) {
        for i in 0..quantity {
            let pos_x: Coord = rng.gen_range(0.0..WORLD_WIDTH);
            let pos_y: Coord = rng.gen_range(0.0..WORLD_HEIGHT);
            let heading = rng.gen_range(0.0..std::f32::consts::TAU);
            let (heading_x, heading_y) = Coord::sin_cos(heading);
            self.bp.push(pos_x, pos_y, heading_x, heading_y);
            self.qt.push(i, &self.bp).unwrap();
        }
    }

    fn calculate_headings(&mut self) {
        const TOO_CLOSE: Coord = 2.5;
        const VIEW_RANGE: Coord = 25.0;
        const SEPARATION_FACTOR: Coord = 0.000;
        const ALIGNMENT_FACTOR: Coord = 0.000;
        const COHESION_FACTOR: Coord = 0.000;
        const CENTER_PULL_FACTOR: Coord = 0.002;

        for id in 0..self.bp.len() {
            self.bp.future_heading_set(id, 0.0, 0.0);

            let position = self.bp.get_position(id);
            let distance_to_center = position.sq_distance_to(&WORLD_CENTER);
            let (dx, dy) = (WORLD_CENTER.x - position.x, WORLD_CENTER.y - position.y);
            let towards_center = Vector2D::new(dx, dy);
            let neighborhood =
                RectContainer::new_from_center(position.x, position.y, VIEW_RANGE, VIEW_RANGE);

            let neighbors = self.qt.query_range(&neighborhood, &self.bp);

            if neighbors.is_empty() {
                dbg!(id, position, neighborhood);
                dbg!(&self);
                panic!("Somehow neighbors is empty...")
            }

            if neighbors.len() == 1 {
                // = 1 because neighbors should include the element itself.
                continue;
            }

            assert!(neighbors.len() >= 2);

            let mut working_space = Vector2D::new(0.0, 0.0);

            // Separation
            for &neighbor in neighbors.iter() {
                if id == neighbor {
                    continue;
                }

                let other_position = self.bp.get_position(neighbor);
                let sq_distance = position.sq_distance_to(other_position);
                if sq_distance < TOO_CLOSE * TOO_CLOSE {
                    working_space.add(position);
                    working_space.sub(other_position);
                }
            }
            self.bp
                .future_heading_add_scaled(id, &working_space, SEPARATION_FACTOR);

            // Alignment
            working_space.x = 0.0;
            working_space.y = 0.0;
            for &neighbor in neighbors.iter() {
                if id == neighbor {
                    continue;
                }

                let heading = self.bp.get_heading(neighbor);

                working_space.add(heading);
            }
            working_space.x /= (neighbors.len() - 1) as Coord;
            working_space.y /= (neighbors.len() - 1) as Coord;
            self.bp
                .future_heading_add_scaled(id, &working_space, ALIGNMENT_FACTOR);

            // Cohesion
            working_space.x = 0.0;
            working_space.y = 0.0;
            for &neighbor in neighbors.iter() {
                if id == neighbor {
                    continue;
                }

                let position = self.bp.get_position(neighbor);

                working_space.add(position);
            }

            working_space.div_scalar((neighbors.len() - 1) as Coord);
            self.bp
                .future_heading_add_scaled(id, &working_space, COHESION_FACTOR);

            // TODO: Pull to center/away from edges
            working_space.x = 0.0;
            working_space.y = 0.0;
            working_space.add_scaled(
                &towards_center,
                CENTER_PULL_FACTOR * distance_to_center * distance_to_center,
            );
        }
    }

    fn move_boids(&mut self, deltatime: f32) {
        const MIN_SPEED: Coord = 1.5;
        const MAX_SPEED: Coord = 3.0;
        const EXPECTED_FPS: Coord = 60.0;
        const TURNING_RATE: Coord = 0.5;

        for id in 0..self.bp.len() {
            let mut future_heading = *self.bp.get_future_heading(id);
            future_heading
                .mult_scalar(TURNING_RATE)
                .add_scaled(self.bp.get_heading(id), 1.0 - TURNING_RATE)
                .clamp(MIN_SPEED, MAX_SPEED)
                .mult_scalar(deltatime * EXPECTED_FPS);

            assert!(!future_heading.x.is_nan());
            assert!(!future_heading.y.is_nan());

            let position = self.bp.get_position(id);
            let mut new_position_x = position.x + future_heading.x;
            let mut new_position_y = position.y + future_heading.y;

            new_position_x = wrap_around(new_position_x, 0.0, WORLD_WIDTH);
            new_position_y = wrap_around(new_position_y, 0.0, WORLD_HEIGHT);
            assert!(!new_position_x.is_nan());
            assert!(!new_position_y.is_nan());
            assert!((0.0..=WORLD_WIDTH).contains(&new_position_x));
            assert!((0.0..=WORLD_HEIGHT).contains(&new_position_y));

            self.bp.set_heading(id, future_heading.x, future_heading.y);
            self.bp.set_position(id, new_position_x, new_position_y);
        }
    }

    fn update(&mut self, deltatime: f32) {
        self.calculate_headings();
        self.move_boids(deltatime);
        self.qt.rebalance(&self.bp);
    }
}

fn wrap_around(x: Coord, min: Coord, max: Coord) -> Coord {
    let mut res = x;
    debug_assert!(max > min);
    let gap = max - min;
    while res < min {
        res += gap
    }
    while res > max {
        res -= gap
    }
    res
}

fn main() {
    // Setup
    let mut world = BoidWorld::new(WORLD_WIDTH, WORLD_HEIGHT);
    let mut rng = thread_rng();
    world.populate(20, &mut rng);

    // Game loop
    for i in 0..1000 {
        // dbg!(&world);
        println!("Iteration: {}", i);
        world.update(1.0 / 60.0);
    }

    println!("Finished.")
}
