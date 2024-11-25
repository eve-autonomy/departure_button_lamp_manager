## PR Type

<!-- Select one and remove others. If an appropriate one is not listed, please write by yourself. -->

- New Feature
- Improvement
- Bug Fix

## Related Links

<!-- Please write related links to GitHub/Jira/Slack/etc. -->

## Description

<!-- Describe what this PR changes. -->

## Review Procedure

<!-- Explain how to review this PR. -->

## Remarks

<!-- Write remarks as you like if you need them. -->

## Pre-Review Checklist for the PR Author

**PR Author should check the checkboxes below when creating the PR.**

- [ ] Code follows [coding guidelines][coding-guidelines]
- [ ] Assign PR to reviewer
- [ ] (If you added new repositories to `.repos`) set permissions for each repository.
      You need to add the following permissions to each github team in the new repository according to [this][repository-permission-setting-flow].
  - READ:
    - eve-autonomy
    - group_x1_sim
    - eva-mapIV
    - ex-fae
  - WRITE:
    - full-time-employee
    - group_x1_dev
    - group-tech-support

## Checklist for the PR Reviewer

**Reviewers should check the checkboxes below before approval.**

- [ ] Commits are properly organized and messages are according to the guideline
- [ ] Code follows [coding guidelines][coding-guidelines]
- [ ] (Optional) Unit tests have been written for new behavior
- [ ] PR title describes the changes
- [ ] (When added something to `.repos`) Check if proper access rights are set.
      You can check the latest permission setting status of each repository [here][github-repository-status].
      If the repository permissions are insufficient, you need to add the permissions according to [this][repository-permission-setting-flow].

## Post-Review Checklist for the PR Author

**PR Author should check the checkboxes below before merging.**

- [ ] All open points are addressed and tracked via issues or tickets
- [ ] Write [release notes][release-notes]

## CI Checks

- **vcs import**: Required to pass before the merge.
- **Check spelling**: NOT required to pass before the merge. It is up to the reviewer(s). See [here][spell-check-dict] if you want to add some words to the spell check dictionary.

[coding-guidelines]: https://tier4.atlassian.net/wiki/spaces/AIP/pages/1194394777/T4
[release-notes]: https://tier4.atlassian.net/wiki/spaces/AIP/pages/563774416
[spell-check-dict]: https://github.com/tier4/autoware-spell-check-dict#how-to-contribute

<!-- Additional links -->

[github-repository-status]: https://docs.google.com/spreadsheets/d/13L1kVWpU5aI_sQIRCR_xk-DuVAK-B4lqkY_xAPWJ-MY/edit#gid=0
[repository-permission-setting-flow]: https://tier4.atlassian.net/wiki/spaces/T4MANUALS/pages/930546509/GitHub+Flow
