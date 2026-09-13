load("//tools/workspace:github.bzl", "github_archive")

def coal_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coal-library/coal",
        upgrade_type = "release",
        commit = "v3.0.4",
        sha256 = "0a4f58e55b88a3d9f873ce79979aecc08491d212ea0b9311df013df3b22bffac",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/no_boost_math.patch",
            ":patches/quoted_includes.patch",
        ],
    )
