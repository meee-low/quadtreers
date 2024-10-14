// @ts-check
"use strict";

const QT_CAPACITY = 12;
const WORLD_WIDTH = 800.0;
const WORLD_HEIGHT = 800.0;
const WORLD_CENTER = [WORLD_WIDTH / 2.0, WORLD_HEIGHT / 2.0];

class RectContainer {
  /**
   *
   * @param {number} lx
   * @param {number} rx
   * @param {number} ty
   * @param {number} by
   */
  constructor(lx, rx, ty, by) {
    this.lx = lx;
    this.rx = rx;
    this.ty = ty;
    this.by = by;
  }

  /**
   *
   * @param {number} cx
   * @param {number} cy
   * @param {number} width
   * @param {number} height
   * @returns {RectContainer}
   */
  static new_from_center(cx, cy, width, height) {
    let half_width = width / 2.0;
    let half_height = height / 2.0;
    return new RectContainer(
      cx - half_width,
      cx + half_width,
      cy - half_height,
      cy + half_height
    );
  }

  /**
   *
   * @param {number} x
   * @param {number} y
   * @returns {boolean}
   */
  contains(x, y) {
    let x_inside = this.lx <= x && x <= this.rx;
    let y_inside = this.ty <= y && y <= this.by;
    return x_inside && y_inside;
  }

  /**
   *
   * @param {RectContainer} other
   * @returns {boolean}
   */
  intersects(other) {
    let no_x_overlap = this.rx < other.lx || other.rx < this.lx;
    let no_y_overlap = this.by < other.ty || other.by < this.ty;

    return !(no_x_overlap || no_y_overlap);
  }
}

class Quadtree {
  /**
   *
   * @param {number} lx
   * @param {number} rx
   * @param {number} ty
   * @param {number} by
   */
  constructor(lx, rx, ty, by) {
    this.container = new RectContainer(lx, rx, ty, by);
    /** @type {number[]} */
    this.boids = [];
    this.subdivisions = null;
  }

  /**
   *
   * @param {number} boid_id
   * @param {BoidPool} bp
   */
  push(boid_id, bp) {
    // Safety check to see if this is the correct quadtree to add to.
    let [posX, posY] = bp.get_position(boid_id);
    if (!this.container.contains(posX, posY)) {
      throw new Error(
        `The boid with id {boid_id} in position \`{position}\` cannot go in the quadtree bounded by "{this.container}".`
      );
    }

    this.push_unchecked_with_position(boid_id, bp, posX, posY);
  }

  /**
   *
   * @param {number} boid_id
   * @param {BoidPool} bp
   * @param {number} posX
   * @param {number} posY
   * @returns
   */
  push_unchecked_with_position(boid_id, bp, posX, posY) {
    if (this.subdivisions === null) {
      if (this.boids.length < QT_CAPACITY) {
        // If there is still room, push here.
        this.boids.push(boid_id);
        return;
      } else {
        // Else, there's not enough room, so we must subdivide if we haven't already.
        this.subdivide(bp);
      }
    }

    // Then add to the matching child.
    let quad = this.find_which_child(posX, posY);

    if (this.subdivisions === null) {
      throw new Error("Subdivisions should never be null at this point.");
    }
    this.subdivisions[quad].push(boid_id, bp);
  }

  /**
   *
   * @param {BoidPool} bp
   */
  subdivide(bp) {
    this.subdivisions = subdivisions_from_parent_container(this.container);

    let boids_ids = [...this.boids];
    this.boids = [];

    for (const boid_id of boids_ids) {
      let [posX, posY] = bp.get_position(boid_id);
      let quad = this.find_which_child(posX, posY);

      this.subdivisions[quad].push(boid_id, bp);
    }
  }

  /**
   *
   * @param {number} posX
   * @param {number} posY
   * @returns {"ne" | "nw" | "se" | "sw"}
   */
  find_which_child(posX, posY) {
    let cx = (this.container.lx + this.container.rx) / 2.0;
    let cy = (this.container.ty + this.container.by) / 2.0;
    let hor = posX <= cx;
    let ver = posY <= cy;

    if (hor && ver) {
      return "nw";
    } else if (hor && !ver) {
      return "sw";
    } else if (!hor && ver) {
      return "ne";
    } else {
      return "se";
    }
  }

  /**
   *
   * @param {BoidPool} bp
   */
  rebalance(bp) {
    let unable_to_rebalance = this._rebalance(bp);
    if (unable_to_rebalance.length > 0) {
      console.error(unable_to_rebalance);
      let positions = unable_to_rebalance.map((id) => bp.get_position(id));
      console.error("Positions: ", positions);
      throw new Error("Could not rebalance properly.");
    }
  }

  /**
   *
   * @param {BoidPool} bp
   * @returns {number[]}
   */
  _rebalance(bp) {
    let rejects = [];

    if (this.subdivisions !== null) {
      // Go deep first and throw the bad ones up.
      rejects.push(...this.subdivisions.ne._rebalance(bp));
      rejects.push(...this.subdivisions.nw._rebalance(bp));
      rejects.push(...this.subdivisions.se._rebalance(bp));
      rejects.push(...this.subdivisions.sw._rebalance(bp));
      if (subdivisions_are_all_empty(this.subdivisions)) {
        this.subdivisions = null;
      }
      // Try to push to another child. Keep the ones that couldn't be pushed to bubble further up.
      rejects = rejects.filter((id) => {
        try {
          this.push(id, bp);
          return false;
        } catch (error) {
          return true;
        }
      });
    } else {
      // Not subdivided, so must check this's own boids.
      this.boids = this.boids.filter((id) => {
        let [posX, posY] = bp.get_position(id);
        if (this.container.contains(posX, posY)) {
          return true;
        } else {
          rejects.push(id);
          return false;
        }
      });
    }

    return rejects;
  }

  /**
   *
   * @param {RectContainer} query
   * @param {BoidPool} bp
   * @returns {number[]}
   */
  query_range(query, bp) {
    // console.debug("Checking the query: ", JSON.stringify(query));
    let results = [];

    if (!this.container.intersects(query)) {
      // console.debug("Quadtree does not intersect the query. Returning...")
      return results;
    }

    if (this.subdivisions) {
      // console.debug("Checking the query within the subdivisions.")
      results.push(...this.subdivisions.ne.query_range(query, bp));
      results.push(...this.subdivisions.nw.query_range(query, bp));
      results.push(...this.subdivisions.se.query_range(query, bp));
      results.push(...this.subdivisions.sw.query_range(query, bp));
    } else {
      for (const id of this.boids) {
        let [posX, posY] = bp.get_position(id);
        if (query.contains(posX, posY)) {
          // console.debug(`Found point ${id} with coordinates ${[posX, posY]} that matches the query.`)
          results.push(id);
        } else {
          // console.debug(`The point ${id} with coordinates ${[posX, posY]} is not within the query, so it won't be added.`)
        }
      }
    }

    return results;
  }

  /**
   *
   * @returns {boolean}
   */
  is_empty() {
    return this.subdivisions === null && this.boids.length === 0;
  }
}

/**
 *
 * @param {RectContainer} container
 * @returns {{ne: Quadtree, nw: Quadtree, sw: Quadtree, se: Quadtree}}
 */
function subdivisions_from_parent_container(container) {
  let cx = (container.lx + container.rx) / 2.0;
  let cy = (container.ty + container.by) / 2.0;
  let ne = new Quadtree(cx, container.rx, container.ty, cy);
  let nw = new Quadtree(container.lx, cx, container.ty, cy);
  let sw = new Quadtree(container.lx, cx, cy, container.by);
  let se = new Quadtree(cx, container.rx, cy, container.by);

  return {
    ne,
    nw,
    sw,
    se,
  };
}

/**
 *
 * @param {{ne: Quadtree, nw: Quadtree, se: Quadtree, sw: Quadtree}} subdivisions
 * @returns {boolean}
 */
function subdivisions_are_all_empty(subdivisions) {
  return (
    subdivisions.ne.is_empty() &&
    subdivisions.nw.is_empty() &&
    subdivisions.se.is_empty() &&
    subdivisions.sw.is_empty()
  );
}

class BoidPool {
  constructor() {
    this.positions = [];
    this.current_headings = [];
    this.calculated_headings = [];
  }

  /**
   *
   * @param {number} pos_x
   * @param {number} pos_y
   * @param {number} heading_x
   * @param {number} heading_y
   */
  push(pos_x, pos_y, heading_x, heading_y) {
    this.positions.push(pos_x, pos_y);
    this.current_headings.push(heading_x, heading_y);
    this.calculated_headings.push(0.0, 0.0);
  }

  /**
   *
   * @param {number} id
   * @returns {[number, number]}
   */
  get_position(id) {
    return [this.positions[id * 2], this.positions[id * 2 + 1]];
  }

  /**
   *
   * @param {number} id
   * @param {number} x
   * @param {number} y
   */
  set_position(id, x, y) {
    this.positions[id * 2] = x;
    this.positions[id * 2 + 1] = y;
  }

  /**
   *
   * @param {number} id
   * @returns {[number, number]}
   */
  get_heading(id) {
    return [this.current_headings[id * 2], this.current_headings[id * 2 + 1]];
  }

  /**
   *
   * @param {number} id
   * @param {number} x
   * @param {number} y
   */
  set_heading(id, x, y) {
    this.current_headings[id * 2] = x;
    this.current_headings[id * 2 + 1] = y;
  }

  /**
   *
   * @param {number} id
   * @returns {[number, number]}
   */
  get_future_heading(id) {
    return [
      this.calculated_headings[id * 2],
      this.calculated_headings[id * 2 + 1],
    ];
  }

  /**
   *
   * @param {number} id
   * @param {number} dx
   * @param {number} dy
   * @param {number} scalar
   */
  future_heading_add_scaled(id, dx, dy, scalar) {
    this.calculated_headings[id * 2] += dx * scalar;
    this.calculated_headings[id * 2 + 1] += dy * scalar;
  }

  /**
   *
   * @param {number} id
   * @param {number} x
   * @param {number} y
   */
  future_heading_set(id, x, y) {
    this.calculated_headings[id * 2] = x;
    this.calculated_headings[id * 2 + 1] = y;
  }

  /**
   *
   * @returns {number}
   */
  len() {
    return this.positions.length / 2;
  }
}

class BoidWorld {
  /**
   *
   * @param {number} width
   * @param {number} height
   */
  constructor(width, height) {
    this.width = width;
    this.height = height;
    this.qt = new Quadtree(0, width * 4, 0, height * 4);
    this.bp = new BoidPool();
  }

  /**
   *
   * @param {number} quantity
   */
  populate(quantity) {
    for (let i = 0; i < quantity; ++i) {
      let posX = Math.random() * this.width;
      let posY = Math.random() * this.height;
      let angle = Math.random() * 2 * Math.PI;
      let headingX = Math.cos(angle);
      let headingY = Math.sin(angle);
      this.bp.push(posX, posY, headingX, headingY);
      this.qt.push(i, this.bp);
    }
  }

  calculate_headings() {
    const TOO_CLOSE = 2.5;
    const VIEW_RANGE = 25.0;
    const SEPARATION_FACTOR = 0.0002;
    const ALIGNMENT_FACTOR = 0.002;
    const COHESION_FACTOR = 0.002;
    const CENTER_PULL_FACTOR = 0.002;

    for (let id = 0; id < this.bp.len(); id++) {
      this.bp.future_heading_set(id, 0.0, 0.0);

      let [posX, posY] = this.bp.get_position(id);
      let neighborhood = RectContainer.new_from_center(
        posX,
        posY,
        VIEW_RANGE,
        VIEW_RANGE
      );

      console.assert(neighborhood.contains(posX, posY), "A RectContainer should contain its center.");

      let neighbors = this.qt.query_range(neighborhood, this.bp);
      if (neighbors.includes(id)) {
        // console.debug("ok, found the id in the neighborhood");
      } else {
        console.assert(
          neighbors.includes(id),
          `The neighborhood should always include oneself. Found: ${JSON.stringify(
            neighbors
          )}, expected: ${id}`
        );
      }

      if (neighbors.length < 2) {
        // 1 is always the boid itself, so it needs at least 2 to make any calculations.
        continue;
      }

      // This vector gets reset and reused for each phase of calculation
      let working_space_x = 0;
      let working_space_y = 0;

      // Separation
      for (const neighbor of neighbors) {
        if (id == neighbor) {
          continue;
        }

        let [otherPosX, otherPosY] = this.bp.get_position(neighbor);
        let dx = posX - otherPosX;
        let dy = posY - otherPosY;
        let sq_distance = dx * dx + dy * dy;
        if (sq_distance < TOO_CLOSE * TOO_CLOSE) {
          working_space_x += posX;
          working_space_y += posY;
          working_space_x -= otherPosX;
          working_space_y -= otherPosY;
        }
      }
      this.bp.future_heading_add_scaled(
        id,
        working_space_x,
        working_space_y,
        SEPARATION_FACTOR
      );

      // Alignment
      working_space_x = 0.0;
      working_space_y = 0.0;
      for (const neighbor of neighbors) {
        if (id === neighbor) {
          continue;
        }

        let [headingX, headingY] = this.bp.get_heading(neighbor);

        // working_space.add(heading);
        working_space_x += headingX;
        working_space_y += headingY;
      }
      // working_space.div_scalar((neighbors.len() - 1));
      working_space_x /= neighbors.length - 1;
      working_space_y /= neighbors.length - 1;
      if (isNaN(working_space_x) || isNaN(working_space_y)) {
        console.assert(isNaN(working_space_x), "Should not be nan");
        console.assert(isNaN(working_space_y), "Should not be nan");
      }
      this.bp.future_heading_add_scaled(
        id,
        working_space_x,
        working_space_y,
        ALIGNMENT_FACTOR
      );

      // Cohesion
      working_space_x = 0.0;
      working_space_y = 0.0;
      for (const neighbor of neighbors) {
        if (id === neighbor) {
          continue;
        }

        let [posX, posY] = this.bp.get_position(neighbor);

        // working_space.add(position);
        working_space_x += posX;
        working_space_y += posY;
      }

      // working_space.div_scalar((neighbors.len() - 1));
      working_space_x /= neighbors.length - 1;
      working_space_y /= neighbors.length - 1;
      if (isNaN(working_space_x) || isNaN(working_space_y)) {
        console.assert(isNaN(working_space_x), "Should not be nan");
        console.assert(isNaN(working_space_y), "Should not be nan");
      }
      this.bp.future_heading_add_scaled(
        id,
        working_space_x,
        working_space_y,
        COHESION_FACTOR
      );

      // Pull away from edges
      let dx = WORLD_CENTER[0] - posX;
      let dy = WORLD_CENTER[1] - posY;
      let distance_to_center = dx * dx + dy * dy;
      this.bp.future_heading_add_scaled(
        id,
        dx,
        dy,
        CENTER_PULL_FACTOR * distance_to_center
      );
    }
  }

  move_boids(deltatime) {
    const MIN_SPEED = 1.5;
    const MAX_SPEED = 3.0;
    const EXPECTED_FPS = 60.0;
    const TURNING_RATE = 0.5;

    for (let id = 0; id < this.bp.len(); id++) {
      let [future_heading_x, future_heading_y] = this.bp.get_future_heading(id);
      // future_heading
      //     .mult_scalar(TURNING_RATE)
      //     .add_scaled(this.bp.get_heading(id), 1.0 - TURNING_RATE)
      //     .clamp(MIN_SPEED, MAX_SPEED)
      //     .mult_scalar(deltatime * EXPECTED_FPS);
      let [cur_heading_x, cur_heading_y] = this.bp.get_heading(id);
      future_heading_x += cur_heading_x;
      future_heading_y += cur_heading_y;
      let sqMag =
        future_heading_x * future_heading_x +
        future_heading_y * future_heading_y;
      if (!(sqMag > 0)) {
        console.assert(sqMag > 0, "Magnitude should be positive.");
      }
      if (sqMag > MAX_SPEED * MAX_SPEED) {
        future_heading_x *= MAX_SPEED / Math.sqrt(sqMag);
        future_heading_y *= MAX_SPEED / Math.sqrt(sqMag);
      } else if (sqMag < MIN_SPEED * MIN_SPEED) {
        future_heading_x *= MIN_SPEED / Math.sqrt(sqMag);
        future_heading_y *= MIN_SPEED / Math.sqrt(sqMag);
      }

      let [posX, posY] = this.bp.get_position(id);
      let new_position_x = posX + future_heading_x;
      let new_position_y = posY + future_heading_y;

      new_position_x = wrap_around(new_position_x, 0.0, WORLD_WIDTH);
      new_position_y = wrap_around(new_position_y, 0.0, WORLD_HEIGHT);

      this.bp.set_heading(id, future_heading_x, future_heading_y);
      this.bp.set_position(id, new_position_x, new_position_y);
    }
  }

  /**
   * @param {number} deltatime
   */
  update(deltatime) {
    this.calculate_headings();
    this.move_boids(deltatime);
    this.qt.rebalance(this.bp);
  }
}

/**
 *
 * @param {number} x
 * @param {number} min
 * @param {number} max
 * @returns {number}
 */
function clamp(x, min, max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  }
  return x;
}

function clampVector(x, y, minMag, maxMag) {
  let sqMag = x * x + y * y;
  if (sqMag < minMag * minMag) {
    let mag = Math.sqrt(sqMag);
    x *= minMag / mag;
    y *= minMag / mag;
  } else if (sqMag > maxMag * maxMag) {
    let mag = Math.sqrt(sqMag);
    x *= maxMag / mag;
    y *= maxMag / mag;
  }
  return [x, y];
}

function wrap_around(x, min, max) {
  let gap = max - min;
  while (x < min) {
    x += gap;
  }
  while (x > max) {
    x -= gap;
  }
  return x;
}

function run_simulation(quantity_of_boids, number_of_iterations) {
  // Setup
  let world = new BoidWorld(WORLD_WIDTH, WORLD_HEIGHT);
  world.populate(quantity_of_boids);

  let clock = performance.now();

  if (number_of_iterations === 0) {
    while (true) {
      world.update(1.0 / 60.0);
    }
  } else {
    for (let _ = 0; _ < number_of_iterations; _++) {
      world.update(1.0 / 60.0);
    }
  }
  return (performance.now() - clock);
}

let main = () => {
  // let iterations = [100];
  // let number_of_boids = [1, 10, 100, 1000];
  let iterations = [1000];
  let number_of_boids = [500];
  console.log(`Target (max) time per frame: ${1000/60}ms.`);

  let min_runs = 5;
  let min_duration = 5 * 1000;
  let max_duration = 120 * 1000;

  for (const it of iterations) {
    for (const num of number_of_boids) {
      let timeSum = 0;
      let current_run = 0;
      while (
        !(
          timeSum >= max_duration ||
          (current_run >= min_runs && timeSum > min_duration)
        )
      ) {
        timeSum += run_simulation(num, it);
        current_run += 1;
      }
      let avg = timeSum / current_run;

      console.log(
        `${num} boids, ${it} frames, ${current_run} runs. Total time: ${(timeSum/1000).toFixed(2)}s. Avg time per frame: ${
          (avg / it).toFixed(3)
        }ms`
      );
    }
    console.log("============");
  }
};

main();
