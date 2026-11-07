import { ChannelStore, type PublishError, type Session, StatusStore } from "@dimos/sdk";
import { TopicClient } from "@dimos/sdk/internal/topics";

export class FakeSession implements Session {
  topics = new TopicClient();
  status = new StatusStore();
  store = new ChannelStore();
  published: [string, unknown][] = [];
  reject: PublishError | null = null;
  rttMs = (): number | null => null;
  watch = () => new Promise<never>(() => {});
  subscribe = () => () => {};
  publish = (ch: string, value: unknown) => {
    this.published.push([ch, value]);
    return this.reject === null
      ? Promise.resolve({ ch, relayTs: 1, bridgeTs: 2 })
      : Promise.reject(this.reject);
  };
  close = () => this.topics.close();
}
