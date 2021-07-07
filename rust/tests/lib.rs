use assert_cmd::Command;
use predicates::prelude::predicate;

#[test]
fn rtcm3tosbp_help() {
    let mut cmd = Command::cargo_bin("rtcm3tosbp").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[test]
fn rtcm3tosbp_convert() {
    let tempfile = tempfile::NamedTempFile::new().unwrap();
    let mut cmd = Command::cargo_bin("rtcm3tosbp").unwrap();
    cmd.arg("-o")
        .arg("-w")
        .arg("2080:123")
        .arg("c/rtcm3tosbp/test/data/piksi-5Hz.rtcm3")
        .arg(tempfile.path())
        .assert()
        .success();
    let stat = std::fs::metadata(tempfile.path()).unwrap();
    assert!(stat.len() > 0);
    let converted = std::fs::File::open(tempfile.path()).unwrap();
    let message_count = sbp::iter_messages(converted).count();
    assert_eq!(message_count, 166);
}

#[test]
fn sbp2rtcm_help() {
    let mut cmd = Command::cargo_bin("sbp2rtcm").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[test]
fn ixcom2sbp_help() {
    let mut cmd = Command::cargo_bin("ixcom2sbp").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[test]
fn ubx2sbp_help() {
    let mut cmd = Command::cargo_bin("ubx2sbp").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[cfg(feature = "nov2sbp")]
#[test]
fn nov2sbp_help() {
    let mut cmd = Command::cargo_bin("nov2sbp").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}
