use ik_geo::nalgebra::{Matrix3, Vector3};
use ik_geo::{
    inverse_kinematics::auxiliary::Kinematics,
    robot::{IKSolver, Robot},
};

fn make_kinematics(h_ptr: *const f64, p_ptr: *const f64) -> Kinematics<6, 7> {
    let mut kin = Kinematics::<6, 7>::new();
    for i in 0..3 {
        for j in 0..6 {
            kin.h[(i, j)] = unsafe { *h_ptr.add(j * 3 + i) };
        }
    }
    for i in 0..3 {
        for j in 0..7 {
            kin.p[(i, j)] = unsafe { *p_ptr.add(j * 3 + i) };
        }
    }

    kin
}

#[no_mangle]
pub extern "C" fn deallocate(robot: *mut Robot) {
    if robot.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(robot));
    }
}

#[no_mangle]
pub extern "C" fn irb6640() -> *mut Robot {
    Box::into_raw(Box::new(ik_geo::robot::irb6640()))
}

#[no_mangle]
pub extern "C" fn ur5() -> *mut Robot {
    Box::into_raw(Box::new(ik_geo::robot::ur5()))
}

#[no_mangle]
pub extern "C" fn three_parallel_bot() -> *mut Robot {
    Box::into_raw(Box::new(ik_geo::robot::three_parallel_bot()))
}

#[no_mangle]
pub extern "C" fn two_parallel_bot() -> *mut Robot {
    Box::into_raw(Box::new(ik_geo::robot::two_parallel_bot()))
}

#[no_mangle]
pub extern "C" fn spherical_bot() -> *mut Robot {
    Box::into_raw(Box::new(ik_geo::robot::spherical_bot()))
}

#[no_mangle]
pub extern "C" fn spherical_two_parallel(h: *const f64, p: *const f64) -> *mut Robot {
    Box::into_raw(Box::new(Robot::spherical_two_parallel(make_kinematics(
        h, p,
    ))))
}

#[no_mangle]
pub extern "C" fn spherical_two_intersecting(h: *const f64, p: *const f64) -> *mut Robot {
    Box::into_raw(Box::new(Robot::spherical_two_intersecting(
        make_kinematics(h, p),
    )))
}

#[no_mangle]
pub extern "C" fn spherical(h: *const f64, p: *const f64) -> *mut Robot {
    Box::into_raw(Box::new(Robot::spherical(make_kinematics(h, p))))
}

#[no_mangle]
pub extern "C" fn three_parallel_two_intersecting(h: *const f64, p: *const f64) -> *mut Robot {
    Box::into_raw(Box::new(Robot::three_parallel_two_intersecting(
        make_kinematics(h, p),
    )))
}

#[no_mangle]
pub extern "C" fn three_parallel(h: *const f64, p: *const f64) -> *mut Robot {
    Box::into_raw(Box::new(Robot::three_parallel(make_kinematics(h, p))))
}

#[no_mangle]
pub extern "C" fn two_parallel(h: *const f64, p: *const f64) -> *mut Robot {
    Box::into_raw(Box::new(Robot::two_parallel(make_kinematics(h, p))))
}

#[no_mangle]
pub extern "C" fn two_intersecting(h: *const f64, p: *const f64) -> *mut Robot {
    Box::into_raw(Box::new(Robot::two_intersecting(make_kinematics(h, p))))
}

#[no_mangle]
pub extern "C" fn gen_six_dof(h: *const f64, p: *const f64) -> *mut Robot {
    Box::into_raw(Box::new(Robot::gen_six_dof(make_kinematics(h, p))))
}

#[no_mangle]
pub extern "C" fn ik(
    robot: *const Robot,
    rotation: *const f64,
    translation: *mut f64,
    n_solutions: *mut usize,
    qs: *mut f64,
    is_ls: *mut bool,
) {
    let robot = unsafe { &*robot };
    let rotation = unsafe { Matrix3::from_row_slice(std::slice::from_raw_parts(rotation, 9)) };
    let translation =
        unsafe { Vector3::from_row_slice(std::slice::from_raw_parts_mut(translation, 3)) };

    let solns = robot.ik(rotation, translation);

    unsafe {
        *n_solutions = solns.len();
        for (i, (q, is_l)) in solns.iter().enumerate() {
            for j in 0..q.len() {
                *qs.add(i * 6 + j) = q[j];
            }
            *is_ls.add(i) = *is_l;
        }
    }
}

#[no_mangle]
pub extern "C" fn fk(
    robot: *const Robot,
    q: *const f64,
    rotation: *mut f64,
    translation: *mut f64,
) {
    let robot = unsafe { &*robot };
    let q = unsafe { std::slice::from_raw_parts(q, 6) };

    let (rot, trans) = robot.fk(q.try_into().unwrap());
    unsafe {
        for i in 0..3 {
            for j in 0..3 {
                *rotation.add(i * 3 + j) = rot[(i, j)];
            }
        }
        for i in 0..3 {
            *translation.add(i) = trans[i];
        }
    }
}
